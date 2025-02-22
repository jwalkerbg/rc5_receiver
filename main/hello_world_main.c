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
#include "soc/soc_caps.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
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

// GPIOs for LEDs
#define LED1_GPIO GPIO_NUM_14
#define LED2_GPIO GPIO_NUM_15
#define LED3_GPIO GPIO_NUM_16

#define CMD_LED1 0x10
#define CMD_LED2 0x20
#define CMD_LED3 0x30

static TaskHandle_t rc5_task_handle = NULL;

rmt_symbol_word_t rc5_buffer[RC5_BUFFER_SIZE];

// Function to initialize LEDs
void init_leds(void) {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED1_GPIO) | (1ULL << LED2_GPIO) | (1ULL << LED3_GPIO),
    };
    gpio_config(&io_conf);
}

rmt_receive_config_t receive_config = {
    .signal_range_min_ns = 1200,
    .signal_range_max_ns = 30 * 1000 * 1000,
};

// LED control
void set_leds(uint8_t cmd) {
    gpio_set_level(LED1_GPIO, (cmd == CMD_LED1) ? 1 : 0);
    gpio_set_level(LED2_GPIO, (cmd == CMD_LED2) ? 1 : 0);
    gpio_set_level(LED3_GPIO, (cmd == CMD_LED3) ? 1 : 0);
}

// Helper function to check if a duration is within tolerance
static bool is_duration_within_tolerance(uint16_t duration, uint16_t expected) {
    return (duration > (expected - RC5_TOLERANCE_US)) &&
           (duration < (expected + RC5_TOLERANCE_US));
}

// Manchester decoding of RC-5 symbols
uint16_t decode_rc5_manchester(const rmt_symbol_word_t *symbols, size_t symbol_count) {
    if (symbol_count < RC5_EXPECTED_BITS * 2) {
        return symbol_count | 0x80; // Error: not enough symbols
    }

    uint16_t bits = 0; // Store decoded bits

    for (size_t i = 0; i < RC5_EXPECTED_BITS; i++) {
        const rmt_symbol_word_t *sym = &symbols[i];

        // Check that the duration is within tolerance
        // if (!is_duration_within_tolerance(sym->duration0, RC5_SYMBOL_DURATION_US / 2) ||
        //     !is_duration_within_tolerance(sym->duration1, RC5_SYMBOL_DURATION_US / 2)) {
        //     return -2;
        // }

        // Manchester decoding:
        // Level transition encodes the bit:
        // 1 -> 0 is Logic '1', 0 -> 1 is Logic '0'
        if (sym->level0 == 1 && sym->level1 == 0) {
            bits = (bits << 1) | 1; // Logic '1'
        } else if (sym->level0 == 0 && sym->level1 == 1) {
            bits = (bits << 1) | 0; // Logic '0'
        } else {
            return -1;
        }
    }

    return bits;
}


// RMT receive done callback

IRAM_ATTR bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    const rmt_rx_done_event_data_t *event_data = (const rmt_rx_done_event_data_t *)edata;
    size_t received_symbols = event_data->num_symbols;

    //ESP_LOGI(TAG, "Received %zu symbols", received_symbols);

    if (received_symbols != 0) {
        rmt_symbol_word_t *symbols = event_data->received_symbols;

        uint16_t rc5_data = decode_rc5_manchester(symbols, received_symbols);
        //if (rc5_data = 0) {
            //ESP_LOGI(TAG, "Decoded RC-5 data: 0x%04X", rc5_data);

            uint8_t command = rc5_data; // Extract the 6-bit command

            // Notify the main task to process the command
            if (rc5_task_handle != NULL) {
                xTaskNotify(rc5_task_handle, command, eSetValueWithOverwrite);
            }
        //}
    }

    // Re-start the receiver for the next packet
    esp_err_t err = rmt_receive(channel, rc5_buffer, sizeof(rc5_buffer), &receive_config);
    if (err != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to re-enable RMT receive: %s", esp_err_to_name(err));
    }

    return false; // No need to yield to a higher-priority task
}

// Main task to handle received RC-5 commands
void rc5_receive_task(void *arg) {
    uint32_t cmd;
    while (true) {
        if (xTaskNotifyWait(0, 0, &cmd, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Handling command: 0x%04lX", cmd);
            set_leds(cmd);
        }
    }
}

// Application main
void app_main(void)
{
    // Configure LEDs
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED1_GPIO) | (1ULL << LED2_GPIO) | (1ULL << LED3_GPIO)
    };
    gpio_config(&io_conf);

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

    xTaskCreate(rc5_receive_task, "rc5_receive_task", 4096, NULL, 10, &rc5_task_handle);
}
