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
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
//#include "esp_rmt.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esp_log.h"

static char* TAG = "RMT";

// RMT configuration
#define RMT_RX_GPIO GPIO_NUM_4
#define RMT_CLK_RES_HZ 1000000    // 1µs resolution
#define RMT_MEM_BLOCKS 64

// RC-5 Protocol parameters
#define RC5_HALF_BIT_US 889       // Half-bit duration in microseconds
#define RC5_TOLERANCE_US 300      // Tolerance for timing variations
#define RC5_FRAME_BITS 14         // Number of bits in an RC-5 frame
#define RC5_TIMEOUT_MS 20         // Timeout to avoid extra bits

// GPIOs for LEDs
#define LED1_GPIO GPIO_NUM_15
#define LED2_GPIO GPIO_NUM_16
#define LED3_GPIO GPIO_NUM_17

// RC-5 Commands for LED Control
#define CMD_LED1_ON 0x10          // Example RC-5 command for LED1
#define CMD_LED2_ON 0x11          // Example RC-5 command for LED2
#define CMD_LED3_ON 0x12          // Example RC-5 command for LED3

// Function to initialize LEDs
void init_leds(void) {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED1_GPIO) | (1ULL << LED2_GPIO) | (1ULL << LED3_GPIO),
    };
    gpio_config(&io_conf);
}

// Function to set LED states
void set_led_state(uint8_t led_index, bool state) {
    gpio_set_level(LED1_GPIO, (led_index == 1) ? state : 0);
    gpio_set_level(LED2_GPIO, (led_index == 2) ? state : 0);
    gpio_set_level(LED3_GPIO, (led_index == 3) ? state : 0);
}

// RC-5 Receiving and Decoding Task
static void rc5_rx_task(void *arg) {
    rmt_channel_handle_t rx_channel = NULL;

    rmt_rx_channel_config_t rx_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = RMT_RX_GPIO,
        .resolution_hz = RMT_CLK_RES_HZ,
        .mem_block_symbols = RMT_MEM_BLOCKS,
    };

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_config, &rx_channel));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));
    ESP_ERROR_CHECK(rmt_receive_timeout(rx_channel, RC5_TIMEOUT_MS * 1000));

    rmt_symbol_word_t raw_symbols[64];

    while (1) {
        size_t num_symbols = 0;
        esp_err_t ret = rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &num_symbols, 1000);

        if (ret == ESP_OK && num_symbols > 0) {
            ESP_LOGI(TAG, "Captured %d symbols", num_symbols);

            uint16_t rc5_data = 0;
            int bit_count = 0;

            for (int i = 0; i < num_symbols - 1; i++) {
                uint32_t high_time = raw_symbols[i].duration0;
                uint32_t low_time = raw_symbols[i].duration1;
                uint32_t next_high_time = raw_symbols[i + 1].duration0;
                uint32_t next_low_time = raw_symbols[i + 1].duration1;

                if (bit_count < RC5_FRAME_BITS) {
                    // Manchester decoding for RC-5
                    if (high_time > RC5_HALF_BIT_US - RC5_TOLERANCE_US &&
                        high_time < RC5_HALF_BIT_US + RC5_TOLERANCE_US &&
                        next_low_time > RC5_HALF_BIT_US - RC5_TOLERANCE_US &&
                        next_low_time < RC5_HALF_BIT_US + RC5_TOLERANCE_US) {
                        rc5_data = (rc5_data << 1) | 1;
                    } else if (low_time > RC5_HALF_BIT_US - RC5_TOLERANCE_US &&
                               low_time < RC5_HALF_BIT_US + RC5_TOLERANCE_US &&
                               next_high_time > RC5_HALF_BIT_US - RC5_TOLERANCE_US &&
                               next_high_time < RC5_HALF_BIT_US + RC5_TOLERANCE_US) {
                        rc5_data = (rc5_data << 1) | 0;
                    }
                    bit_count++;
                }
            }

            if (bit_count == RC5_FRAME_BITS) {
                uint8_t start_bits = (rc5_data >> 12) & 0x03;
                uint8_t toggle_bit = (rc5_data >> 11) & 0x01;
                uint8_t address = (rc5_data >> 6) & 0x1F;
                uint8_t command = rc5_data & 0x3F;

                ESP_LOGI(TAG, "RC-5 Received - Start: %02X, Toggle: %d, Address: %02X, Command: %02X",
                         start_bits, toggle_bit, address, command);

                // Handle recognized commands
                switch (command) {
                    case CMD_LED1_ON:
                        set_led_state(1, true);
                        break;
                    case CMD_LED2_ON:
                        set_led_state(2, true);
                        break;
                    case CMD_LED3_ON:
                        set_led_state(3, true);
                        break;
                    default:
                        ESP_LOGW(TAG, "Unknown command");
                        break;
                }
            } else {
                ESP_LOGW(TAG, "Invalid RC-5 frame (Only %d bits)", bit_count);
            }
        }
    }
}

void app_main(void) {
    init_leds();
    xTaskCreate(rc5_rx_task, "rc5_rx_task", 4096, NULL, 5, NULL);
}









/*
void app_main(void)
{
    ESP_LOGI(TAG,"Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG,"This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG,"silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGI(TAG,"Get flash size failed");
        return;
    }

    ESP_LOGI(TAG,"%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG,"Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--) {
        ESP_LOGI(TAG,"Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI("Restarting now.\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    esp_restart();
}
*/