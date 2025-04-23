/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "rc5_receiver.h"

static char* TAG = "RC5APP";

static QueueHandle_t rc5_queue = NULL;

// RC5 handler
void rc5_handler(void* param)
{
    rc5_context_t rc5;
    ESP_LOGI(TAG, "RC5 handler started");
    QueueHandle_t q = (QueueHandle_t)param;
    while (true) {
        if (xQueueReceive(q, &rc5, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "RC5 frame: 0x%04X, command: 0x%02X, address: 0x%02X, toggle: %d, start: %d, event: %s",
                rc5.rc5_data.frame, rc5.rc5_data.command, rc5.rc5_data.address, rc5.rc5_data.toggle, rc5.rc5_data.start,rc5_event_name(rc5.event));
            if (rc5.event == RC5_EVENT_INVAID) {
                break;
            }
        }
    }
    ESP_LOGI(TAG, "RC5 handler terminated");
    vTaskDelete(NULL);
}

// Application main
void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    rc5_queue = xQueueCreate(10, sizeof(rc5_context_t));
    rc5_receiver_init(rc5_queue);
}
