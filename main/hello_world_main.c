/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_log.h"

#include "rc5_receiver.h"

static char* TAG = "RC5APP";

// RC5 handler
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

    vTaskDelay(pdMS_TO_TICKS(5000));
    rc5_setup(rc5_handler);
}
