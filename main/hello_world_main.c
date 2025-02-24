/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_types.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esp_log.h"

static char* TAG = "RMT";

// RMT configuration
#define RMT_RX_GPIO GPIO_NUM_4
#define RMT_CLK_RES_HZ 1000000    // 1µs resolution
#define RC5_BUFFER_SIZE 64
#define RC5_EXPECTED_BITS 14
#define RC5_SYMBOL_DURATION_US 889       // Manchester symbol duration (approximately 889 µs)
#define RC5_TOLERANCE_US 200             // Tolerance for signal timing (±200 µs)

typedef union {
    struct {
        uint16_t command:6;
        uint16_t address:5;
        uint16_t toggle:1;
        uint16_t reserved:4;
    };
    uint16_t frame;
} rc5_data_t;   // RC5 command data structure

static TaskHandle_t rc5_task_handle = NULL;

rmt_symbol_word_t rc5_buffer[RC5_BUFFER_SIZE];
rmt_symbol_word_t rc5_buffer_cp[RC5_BUFFER_SIZE];

rmt_receive_config_t receive_config = {
    .signal_range_min_ns = 1200,
    .signal_range_max_ns = 30 * 1000 * 1000,
};

// Helper function to check if a duration is within tolerance
static bool is_duration_within_tolerance(uint16_t duration, uint16_t expected) {
    return (duration > (expected - RC5_TOLERANCE_US)) &&
           (duration < (expected + RC5_TOLERANCE_US));
}

// RMT receive done callback

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

typedef void (*rc5_handler_t)(rc5_data_t rc5_data);

static rc5_data_t rc5_decoder(rmt_symbol_word_t* symbols, size_t count);

// Main task to handle received RC-5 commands
void rc5_receive_task(void *arg)
{
    uint32_t num_symbols;
    rc5_handler_t rc5h = (rc5_handler_t)arg;

    ESP_LOGI(TAG, "RC5 receive task started");
    while (true) {
        if (xTaskNotifyWait(0, 0, &num_symbols, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Handling %lu symbols", num_symbols);
            for (int i = 0; i < num_symbols; i++) {
                ESP_LOGD(TAG, "Symbol %d: %04lX: %4d %4d %d %d", i, rc5_buffer_cp[i].val, rc5_buffer_cp[i].duration0, rc5_buffer_cp[i].duration1, rc5_buffer_cp[i].level0, rc5_buffer_cp[i].level1);
            }
            rc5_data_t rc5_data = rc5_decoder(rc5_buffer_cp, num_symbols);

            if (rc5h != NULL) {
                rc5h(rc5_data);
            }
        }
    }
    ESP_LOGI(TAG, "RC5 receive task ended");
    vTaskDelete(NULL);
}

static rc5_data_t rc5_decoder(rmt_symbol_word_t* symbols, size_t count)
{
    rc5_data_t rc_data = { 0 };
    bool accept_bit = true;
    for (int i = 0; i < count; i++) {
        if (accept_bit) {
            rc_data.frame = (rc_data.frame << 1) | symbols[i].level0;
            if (symbols[i].duration0 < 1050) {
                accept_bit = false;
            }
        }
        else {
            accept_bit = true;
        }

        if (accept_bit) {
            rc_data.frame = (rc_data.frame << 1) | symbols[i].level1;
            if (symbols[i].duration1 < 1050) {
                accept_bit = false;
            }
        }
        else {
            accept_bit = true;
        }
    }

    return rc_data;
}

// Set the RC5 handler
void rc5_handler(rc5_data_t rc5_data)
{
    ESP_LOGI(TAG, "RC5 frame: 0x%04X, command: 0x%02X, address: 0x%02X, toggle: %d", rc5_data.frame, rc5_data.command, rc5_data.address, rc5_data.toggle);
}

// Application main
void app_main(void)
{
    gpio_config_t io_rmt_conf = {
        .pin_bit_mask = 1ULL << RMT_RX_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_rmt_conf));

    rmt_rx_channel_config_t rx_config = {
        .clk_src = RMT_CLK_SRC_APB,
        .gpio_num = RMT_RX_GPIO,
        .resolution_hz = RMT_CLK_RES_HZ, // 1us resolution
        .mem_block_symbols = RC5_BUFFER_SIZE,
        .flags = {
            .invert_in = 1,
            .with_dma = 0,
            .io_loop_back = 0,
            .allow_pd = 0,
        },
    };

    rmt_channel_handle_t rx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_config, &rx_channel));

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback, // Set the callback function
    };

    // ESP_LOGI(TAG, "rx_channel = %p, cbs = %p", rx_channel, &cbs);
    // do {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // } while(1);

    vTaskDelay(pdMS_TO_TICKS(10000));

    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, NULL));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));
    ESP_ERROR_CHECK(rmt_receive(rx_channel, rc5_buffer, sizeof(rc5_buffer), &receive_config));

    xTaskCreate(rc5_receive_task, "rc5_receive_task", 4096, rc5_handler, 10, &rc5_task_handle);
}
