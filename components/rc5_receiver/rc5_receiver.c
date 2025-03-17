// rc5_receiver.c

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rmt_types.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"

#include "rc5_receiver.h"

static char* TAG = "RC5RCV";

// RMT configuration
#define RMT_RX_GPIO CONFIG_RC5_RX_GPIO                              // GPIO pin for RMT receiver
#define RC5_INVERT_IN CONFIG_RC5_INVERT_IN                          // Invert input signal
#define RMT_CLK_RES_HZ CONFIG_RMT_CLK_RES_HZ                        // Resolution
#define RC5_BUFFER_SIZE CONFIG_RC5_BUFFER_SIZE                      // RMT buffer size
#define RC5_SYMBOL_DURATION_US CONFIG_RC5_SYMBOL_DURATION_US        // Manchester symbol duration (approximately 889 µs)
#define RC5_TOLERANCE_US CONFIG_RC5_TOLERANCE_US                    // Tolerance for signal timing (±200 µs)
#define RC5_LONG_PRESS_THRESHOLD CONFIG_RC5_LONG_PRESS_THRESHOLD    // Number of frames before detecting a long press
#define RC5_FRAME_INTERVAL pdMS_TO_TICKS(CONFIG_RC5_FRAME_INTERVAL) // 107ms frame rate

#define RC5_TERMINATE (0xffff)

// RC5 state structure
typedef struct {
    rc5_data_t last_button;
    uint8_t frame_count;
    bool long_press_active;
    rmt_channel_handle_t rx_channel;
    TaskHandle_t rc5_task_handle;
    TimerHandle_t timer;
    rc5_handler_t event_callback;
    SemaphoreHandle_t mutex;
} rc5_state_t;

static rc5_state_t rc5_state = { 0 };

rmt_symbol_word_t rc5_buffer[RC5_BUFFER_SIZE];
rmt_symbol_word_t rc5_buffer_cp[RC5_BUFFER_SIZE];

rmt_receive_config_t receive_config = {
    .signal_range_min_ns = 1200,
    .signal_range_max_ns = 30 * 1000 * 1000,
};

static rc5_data_t rc5_decoder(rmt_symbol_word_t* symbols, size_t count);
static void rc5_event_handler(rc5_data_t rc5_data);

// RMT receiver callback function
// This function is called when the RMT receiver has received a packet.
// It copies the received symbols to a buffer and notifies the main task to process the command.
// The main task will then decode the RC-5 command and call the user-defined handler.

static IRAM_ATTR bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    const rmt_rx_done_event_data_t *event_data = (const rmt_rx_done_event_data_t *)edata;
    size_t num_symbols = event_data->num_symbols;

    if (num_symbols != 0) {
        rmt_symbol_word_t *received_symbols = event_data->received_symbols;

        for (int i = 0; i < num_symbols; i++) {
            rc5_buffer_cp[i] = received_symbols[i];
        }

        // Notify the main task to process the command
        if (rc5_state.rc5_task_handle != NULL) {
            xTaskNotifyFromISR(rc5_state.rc5_task_handle, num_symbols, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
        }
    }

    // Re-start the receiver for the next packet
    rmt_receive(channel, rc5_buffer, sizeof(rc5_buffer), &receive_config);

    return pxHigherPriorityTaskWoken == pdTRUE; // No need to yield to a higher-priority task
}

// RC5 receive task
// This task is responsible for processing the received RC-5 commands.
// It waits for a notification from the RMT receiver callback and decodes the RC-5 command.
// The decoded command is then passed to the user-defined handler.

static void rc5_receive_task(void *arg)
{
    uint32_t num_symbols;

    ESP_LOGI(TAG, "RC5 receive task started");
    while (true) {
        if (xTaskNotifyWait(0, 0, &num_symbols, portMAX_DELAY)) {
            if (num_symbols == RC5_TERMINATE) {
                break;
            }
            ESP_LOGD(TAG, "Handling %lu symbols", num_symbols);
            for (int i = 0; i < num_symbols; i++) {
                ESP_LOGD(TAG, "Symbol %d: %04lX: %4d %4d %d %d", i, rc5_buffer_cp[i].val, rc5_buffer_cp[i].duration0, rc5_buffer_cp[i].duration1, rc5_buffer_cp[i].level0, rc5_buffer_cp[i].level1);
            }
            rc5_data_t rc5_data = rc5_decoder(rc5_buffer_cp, num_symbols);

            if (rc5_data.start != 3) {
                // Invalid command
                ESP_LOGD(TAG, "Invalid command");
                continue;
            }
            rc5_event_handler(rc5_data);
        }
    }
    ESP_LOGI(TAG, "RC5 receive task ended");
    vTaskDelete(NULL);
}

// RC5 decoder
// This function decodes the received RC-5 symbols and returns the decoded command.
// The RC-5 protocol uses Manchester encoding, where a logical 0 is represented by a short high pulse followed by a long low pulse,
// and a logical 1 is represented by a long high pulse followed by a short low pulse.

static rc5_data_t rc5_decoder(rmt_symbol_word_t* symbols, size_t count)
{
    rc5_data_t rc_data = { 0 };
    bool accept_bit = true;
    for (int i = 0; i < count; i++) {
        if (accept_bit) {
            rc_data.frame = (rc_data.frame << 1) | symbols[i].level0;
            if (symbols[i].duration0 < (RC5_SYMBOL_DURATION_US + RC5_TOLERANCE_US)) {
                accept_bit = false;
            }
        }
        else {
            accept_bit = true;
        }

        if (accept_bit) {
            rc_data.frame = (rc_data.frame << 1) | symbols[i].level1;
            if (symbols[i].duration1 < (RC5_SYMBOL_DURATION_US + RC5_TOLERANCE_US)) {
                accept_bit = false;
            }
        }
        else {
            accept_bit = true;
        }
    }

    return rc_data;
}

// State variables for auto-repeat functionality

// Timer callback to handle missing frames (button release detection)

// void rc5_timer_callback(TimerHandle_t xTimer)
// Input:
//   xTimer: Timer handle
// Description:  This function is called when the timer expires.
// It is used to detect the end of a button press (short press) or a long press.
// If a button press is detected, it triggers the appropriate event (short press or long press end).
// If no button press is detected, it resets the state.

static void rc5_timer_callback(TimerHandle_t xTimer)
{
    if (xSemaphoreTake(rc5_state.mutex, portMAX_DELAY)) {  // Lock mutex
        if (rc5_state.last_button.frame != 0) {  // A button was active
            if (rc5_state.long_press_active) {
                rc5_state.event_callback(RC5_EVENT_LONG_PRESS_END, rc5_state.last_button);
            } else {
                rc5_state.event_callback(RC5_EVENT_SHORT_PRESS, rc5_state.last_button);
            }
            // Reset state
            rc5_state.last_button.frame = 0;
            rc5_state.frame_count = 0;
            rc5_state.long_press_active = false;
        }
        xSemaphoreGive(rc5_state.mutex);  // Unlock mutex
    }
}

// static void rc5_event_handler(rc5_data_t rc5_data)
// Input:
//   rc5_data: Decoded RC5 command data
// Description:  This function processes the decoded RC5 command data and triggers the appropriate event.
// It handles short press, long press start, and long press end events.
// The event is determined based on the number of frames received and the timing of the frames.
// It works in conjunction with the timer to detect long press events.
static void rc5_event_handler(rc5_data_t rc5_data)
{
    if (xSemaphoreTake(rc5_state.mutex, portMAX_DELAY)) {  // Lock mutex
        if (rc5_data.command != rc5_state.last_button.command) {
            // New button press detected, we need to reset the state
            rc5_state.last_button = rc5_data;
            rc5_state.frame_count = 1;
            rc5_state.long_press_active = false;
        }
        else {
            // Continue counting frames
            rc5_state.frame_count++;
            if (!rc5_state.long_press_active && rc5_state.frame_count >= RC5_LONG_PRESS_THRESHOLD) {
                rc5_state.event_callback(RC5_EVENT_LONG_PRESS_START, rc5_data);
                rc5_state.long_press_active = true;
            }
        }

        // Ensure timer starts or resets properly
        if (xTimerIsTimerActive(rc5_state.timer) == pdFALSE) {
            xTimerStart(rc5_state.timer, 0);
        } else {
            xTimerReset(rc5_state.timer, 0);
        }

        xSemaphoreGive(rc5_state.mutex);  // Unlock mutex
    }
}

// void rc5_set_event_callback(rc5_handler_t callback)
// Input:
//   callback: User-defined callback function to handle RC5 events
// Description:  This function sets the user-defined callback function to handle RC5 events.
// The callback function should have the following signature:
// void rc5_event_handler(rc5_event_t event, rc5_data_t rc5_data);
// The callback function will be called with the following parameters:
//   - event: The type of RC5 event (short press, long press start, long press end)
//   - rc5_data: The decoded RC5 command data
// The callback function should be defined by the user and should handle the RC5 events as needed.
void rc5_set_event_callback(rc5_handler_t callback)
{
    if (xSemaphoreTake(rc5_state.mutex, portMAX_DELAY)) {
        rc5_state.event_callback = callback;
        xSemaphoreGive(rc5_state.mutex);
    }
}

// const char* rc5_event_name(rc5_event_t event)
// Input:
//   event: RC5 event event
// Output:
//   const char*: String representation of the RC5 event type
// Description:  This function takes an RC5 event type (`rc5_event_t`) and returns
// a corresponding string representation. This is useful for logging or debugging purposes.

const char* rc5_event_name(rc5_event_t event)
{
    switch (event) {
        case RC5_EVENT_SHORT_PRESS:
            return "SHORT_PRESS";
        case RC5_EVENT_LONG_PRESS_START:
            return "LONG_PRESS_START";
        case RC5_EVENT_LONG_PRESS_END:
            return "LONG_PRESS_END";
        default:
            return "UNKNOWN";
    }
}

// RC5 setup
// This function initializes the RC5 receiver.
// It configures the RMT receiver channel and starts the receiver.
// It also creates the RC5 receive task to process the received commands.
// The user-defined handler is called with the decoded RC5 command.
esp_err_t rc5_receiver_init(rc5_handler_t rc5_handler)
{
    if (rc5_handler == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    rc5_state.mutex = xSemaphoreCreateMutex();
    if (rc5_state.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex!");
        return ESP_ERR_NO_MEM;
    }

    // Set user-defined callback
    rc5_set_event_callback(rc5_handler);

    gpio_config_t io_rmt_conf = {
        .pin_bit_mask = 1ULL << RMT_RX_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_rmt_conf));

    // Initialize timer
    rc5_state.timer = xTimerCreate("RC5_Timer", RC5_FRAME_INTERVAL * 2, pdFALSE, NULL, rc5_timer_callback);
    if (rc5_state.timer == NULL) {
        ESP_LOGE(TAG, "Failed to create RC5 Event timer.");
        return ESP_ERR_NO_MEM;
    }

    rmt_rx_channel_config_t rx_config = {
        .clk_src = RMT_CLK_SRC_APB,
        .gpio_num = RMT_RX_GPIO,
        .resolution_hz = RMT_CLK_RES_HZ, // 1us resolution
        .mem_block_symbols = RC5_BUFFER_SIZE,
        .flags = {
            .invert_in = RC5_INVERT_IN ? 1 : 0,
            .with_dma = 0,
            .io_loop_back = 0,
            .allow_pd = 0,
        },
    };

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_config, &rc5_state.rx_channel));

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback, // Set the callback function
    };

    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rc5_state.rx_channel, &cbs, NULL));
    ESP_ERROR_CHECK(rmt_enable(rc5_state.rx_channel));
    ESP_ERROR_CHECK(rmt_receive(rc5_state.rx_channel, rc5_buffer, sizeof(rc5_buffer), &receive_config));

    if (xTaskCreate(rc5_receive_task, "rc5_receive_task", 4096, rc5_handler, 10, &rc5_state.rc5_task_handle)  != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RC5 receive task.");

        if (rc5_state.rx_channel != NULL) {
            rmt_disable(rc5_state.rx_channel);
            rmt_del_channel(rc5_state.rx_channel);
        }
        if (rc5_state.timer != NULL) {
            xTimerStop(rc5_state.timer, 0);
            xTimerDelete(rc5_state.timer, 0);
        }

        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

// Terminate the RC5 receiver task
// This function is called to terminate the RC5 receiver task.
// It sends a notification to the task to terminate.
// The task will then exit the loop and delete itself.
void rc5_terminate(void)
{
    rmt_disable(rc5_state.rx_channel);
    rmt_del_channel(rc5_state.rx_channel);
    if (rc5_state.rc5_task_handle != NULL) {
        xTaskNotify(rc5_state.rc5_task_handle, RC5_TERMINATE, eSetValueWithOverwrite);
    }
    if (rc5_state.timer != NULL) {
        xTimerDelete(rc5_state.timer, 0);
    }
    if (rc5_state.mutex != NULL) {
        vSemaphoreDelete(rc5_state.mutex);
    }
}

// end of rc5_receiver.c
