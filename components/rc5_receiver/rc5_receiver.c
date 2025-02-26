// rc5_receiver.c

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rmt_types.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"

#include "rc5_receiver.h"

static char* TAG = "RC5";

#define RC5_TERMINATE (0xffff)

static rmt_channel_handle_t rx_channel = NULL;
static TaskHandle_t rc5_task_handle = NULL;

rmt_symbol_word_t rc5_buffer[RC5_BUFFER_SIZE];
rmt_symbol_word_t rc5_buffer_cp[RC5_BUFFER_SIZE];

rmt_receive_config_t receive_config = {
    .signal_range_min_ns = 1200,
    .signal_range_max_ns = 30 * 1000 * 1000,
};

static rc5_data_t rc5_decoder(rmt_symbol_word_t* symbols, size_t count);
static void rc5_auto_repeat_handler(rc5_data_t rc5_data, rc5_handler_t rc5h);

// RMT receiver callback function
// This function is called when the RMT receiver has received a packet.
// It copies the received symbols to a buffer and notifies the main task to process the command.
// The main task will then decode the RC-5 command and call the user-defined handler.
IRAM_ATTR bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_ctx)
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
        if (rc5_task_handle != NULL) {
            xTaskNotifyFromISR(rc5_task_handle, num_symbols, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
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
//
void rc5_receive_task(void *arg)
{
    uint32_t num_symbols;
    rc5_handler_t rc5h = (rc5_handler_t)arg;

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
            if (rc5h != NULL) {
                rc5_auto_repeat_handler(rc5_data, rc5h);
            }
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
static bool auto_repeat_enabled = RC5_AUTO_REPEAT_ENABLE;
static int repeat_counter = 0;
static int repeat_postscaler = RC5_AUTO_REPEAT_POSTSCALER;
static uint8_t last_toggle_bit = 0xFF;
static uint16_t last_command = 0xFFFF;

// Mutex for thread safety
static SemaphoreHandle_t auto_repeat_mutex = NULL;

// Auto-repeat handler
// This function handles the auto-repeat functionality for RC5 commands.
// It keeps track of the last command and toggle bit received and repeats the command based on the postscaler.
// If the postscaler is set to 1, the command will be repeated indefinitely.
static void rc5_auto_repeat_handler(rc5_data_t rc5_data, rc5_handler_t rc5h)
{
    bool repeat_enabled = false;
    int postscaler = 1;

    // Safely read shared variables
    if (xSemaphoreTake(auto_repeat_mutex, portMAX_DELAY) == pdTRUE) {
        repeat_enabled = auto_repeat_enabled;
        postscaler = repeat_postscaler;
        xSemaphoreGive(auto_repeat_mutex);
    }

    if ((rc5_data.command == last_command) && (rc5_data.toggle == last_toggle_bit)) {
        // Repeated command
        if (repeat_enabled) {
            repeat_counter++;
            if (repeat_counter >= postscaler) {
                rc5h(rc5_data);         // Call app handler on n-th repeat
                repeat_counter = 0;     // Reset repeat counter
            }
        }
    } else {
        // New command
        rc5h(rc5_data);                     // Call app handler immediately
        last_command = rc5_data.command;    // Update last command
        last_toggle_bit = rc5_data.toggle;  // Update toggle bit
        repeat_counter = 0;                 // Reset repeat counter
    }
}

// Set auto-repeat mode
// This function enables or disables the auto-repeat mode for RC5 commands.
// The auto-repeat mode allows the same command to be repeated multiple times with a postscaler.
// The user can set the postscaler to determine how many times the command should be repeated before calling the handler.
// If the postscaler is set to 1, the command will be repeated indefinitely.
void set_auto_repeat(bool enabled, int postscaler)
{
    if (xSemaphoreTake(auto_repeat_mutex, portMAX_DELAY) == pdTRUE) {
        auto_repeat_enabled = enabled;
        repeat_postscaler = postscaler > 0 ? postscaler : 1;    // Avoid invalid postscaler
        xSemaphoreGive(auto_repeat_mutex);
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

    if (auto_repeat_mutex == NULL) {
        auto_repeat_mutex = xSemaphoreCreateMutex();
        if (auto_repeat_mutex == NULL) {
            ESP_LOGD(TAG,"Failed to create mutex!");
        }
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

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_config, &rx_channel));

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback, // Set the callback function
    };

    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, NULL));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));
    ESP_ERROR_CHECK(rmt_receive(rx_channel, rc5_buffer, sizeof(rc5_buffer), &receive_config));

    xTaskCreate(rc5_receive_task, "rc5_receive_task", 4096, rc5_handler, 10, &rc5_task_handle);

    return ESP_OK;
}

// Terminate the RC5 receiver task
// This function is called to terminate the RC5 receiver task.
// It sends a notification to the task to terminate.
// The task will then exit the loop and delete itself.
void rc5_terminate(void)
{
    rmt_disable(rx_channel);
    rmt_del_channel(rx_channel);
    if (rc5_task_handle != NULL) {
        xTaskNotify(rc5_task_handle, RC5_TERMINATE, eSetValueWithOverwrite);
    }
}

// end of rc5_receiver.c
