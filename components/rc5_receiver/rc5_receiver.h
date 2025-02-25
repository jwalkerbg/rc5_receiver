// rc5_receiver.h

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rmt_types.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// RMT configuration
#define RMT_RX_GPIO GPIO_NUM_4
#define RMT_CLK_RES_HZ 1000000    // 1µs resolution
#define RC5_BUFFER_SIZE 64
#define RC5_EXPECTED_BITS 14
#define RC5_SYMBOL_DURATION_US 889       // Manchester symbol duration (approximately 889 µs)
#define RC5_TOLERANCE_US 200             // Tolerance for signal timing (±200 µs)

// RC5 command data structure
typedef union {
    struct {
        uint16_t command:6;
        uint16_t address:5;
        uint16_t toggle:1;
        uint16_t reserved:4;
    };
    uint16_t frame;
} rc5_data_t;   // RC5 command data structure

typedef void (*rc5_handler_t)(rc5_data_t rc5_data);

esp_err_t rc5_setup(rc5_handler_t rc5_handler);

#ifdef __cplusplus
}   // extern "C"
#endif

// end of rc5_receiver.h