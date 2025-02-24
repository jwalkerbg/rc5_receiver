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
#define RC5_TOLERANCE_US 300             // Tolerance for signal timing (±200 µs)

static TaskHandle_t rc5_task_handle = NULL;

rmt_symbol_word_t rc5_buffer[RC5_BUFFER_SIZE];
rmt_symbol_word_t rc5_buffer_cp[RC5_BUFFER_SIZE];
uint32_t scnt = 0;

rmt_receive_config_t receive_config = {
    .signal_range_min_ns = 1200,
    .signal_range_max_ns = 30 * 1000 * 1000,
};

// Helper function to check if a duration is within tolerance
static bool is_duration_within_tolerance(uint16_t duration, uint16_t expected) {
    return (duration > (expected - RC5_TOLERANCE_US)) &&
           (duration < (expected + RC5_TOLERANCE_US));
}

uint16_t decode_rc5_manchester(const rmt_symbol_word_t *symbols, size_t symbol_count)
{
    uint16_t rc5_data = 0;
    int bit_count = 0;

    for (int i = 0; i < symbol_count -1; i++) {
        uint32_t high_time = symbols[i].duration0;
        uint32_t low_time = symbols[i].duration1;
        uint32_t next_high_time = symbols[i + 1].duration0;
        uint32_t next_low_time = symbols[i + 1].duration1;

        if (bit_count < RC5_EXPECTED_BITS) {
            // Manchester decoding for RC-5
            if (high_time > RC5_SYMBOL_DURATION_US - RC5_TOLERANCE_US &&
                high_time < RC5_SYMBOL_DURATION_US + RC5_TOLERANCE_US &&
                next_low_time > RC5_SYMBOL_DURATION_US - RC5_TOLERANCE_US &&
                next_low_time < RC5_SYMBOL_DURATION_US + RC5_TOLERANCE_US) {
                rc5_data = (rc5_data << 1) | 1;
            } else if (low_time > RC5_SYMBOL_DURATION_US - RC5_TOLERANCE_US &&
                       low_time < RC5_SYMBOL_DURATION_US + RC5_TOLERANCE_US &&
                       next_high_time > RC5_SYMBOL_DURATION_US - RC5_TOLERANCE_US &&
                       next_high_time < RC5_SYMBOL_DURATION_US + RC5_TOLERANCE_US) {
                rc5_data = (rc5_data << 1) | 0;
            }
            bit_count++;
        }
        return rc5_data;
    }
    return 0;
}

// Manchester decoding of RC-5 symbols
uint16_t decode_rc5_manchester_(const rmt_symbol_word_t *symbols, size_t symbol_count) {
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

    if (received_symbols != 0) {
        rmt_symbol_word_t *symbols = event_data->received_symbols;

        uint8_t command = 0; //rc5_data; // Extract the 6-bit command
        for (int i = 0; i < received_symbols; i++) {
            rc5_buffer_cp[i] = symbols[i];
        }
        scnt = received_symbols;

        // Notify the main task to process the command
        if (rc5_task_handle != NULL) {
            xTaskNotify(rc5_task_handle, command, eSetValueWithOverwrite);
        }
    }

    // Re-start the receiver for the next packet
    rmt_receive(channel, rc5_buffer, sizeof(rc5_buffer), &receive_config);

    return false; // No need to yield to a higher-priority task
}

// Main task to handle received RC-5 commands
void rc5_receive_task(void *arg) {
    uint32_t cmd;
    while (true) {
        if (xTaskNotifyWait(0, 0, &cmd, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Handling command: %lu", cmd);
            for (int i = 0; i < scnt; i++) {
                ESP_LOGI(TAG, "Symbol %d: %04lX: %4d %4d %d %d", i, rc5_buffer_cp[i].val, rc5_buffer_cp[i].duration0, rc5_buffer_cp[i].duration1, rc5_buffer_cp[i].level0, rc5_buffer_cp[i].level1);
            }
            uint16_t rc_data = 0;
            bool accept_bit = true;
            for (int i = 0; i < scnt; i++) {
                if (accept_bit) {
                    rc_data = (rc_data << 1) | rc5_buffer_cp[i].level0;
                    if (rc5_buffer_cp[i].duration0 < 1050) {
                        accept_bit = false;
                    }
                }
                else {
                    accept_bit = true;
                }

                if (accept_bit) {
                    rc_data = (rc_data << 1) | rc5_buffer_cp[i].level1;
                    if (rc5_buffer_cp[i].duration1 < 1050) {
                        accept_bit = false;
                    }
                }
                else {
                    accept_bit = true;
                }
            }
            uint16_t command = rc_data & 0x3F;
            uint16_t address = (rc_data >> 6) & 0x1F;
            uint16_t toggle = (rc_data >> 11) & 0x1;
            ESP_LOGI(TAG, "RC5 command: frame: 0x%04X, 0x%02X, address: 0x%02X, toggle: %d", rc_data, command, address, toggle);
        }
    }
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

    xTaskCreate(rc5_receive_task, "rc5_receive_task", 4096, NULL, 10, &rc5_task_handle);
}
