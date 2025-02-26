// rc5_receiver.h

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// RMT configuration
#define RMT_RX_GPIO CONFIG_RC5_RX_GPIO                              // GPIO pin for RMT receiver
#define RC5_INVERT_IN CONFIG_RC5_INVERT_IN                          // Invert input signal
#define RMT_CLK_RES_HZ CONFIG_RMT_CLK_RES_HZ                        // Resolution
#define RC5_BUFFER_SIZE CONFIG_RC5_BUFFER_SIZE                      // RMT buffer size
#define RC5_SYMBOL_DURATION_US CONFIG_RC5_SYMBOL_DURATION_US        // Manchester symbol duration (approximately 889 µs)
#define RC5_TOLERANCE_US CONFIG_RC5_TOLERANCE_US                    // Tolerance for signal timing (±200 µs)
#define RC5_AUTO_REPEAT_ENABLE CONFIG_RC5_AUTO_REPEAT_ENABLE        // Enable auto-repeat
#define RC5_AUTO_REPEAT_POSTSCALER CONFIG_RC5_AUTO_REPEAT_POSTSCALER  // Auto-repeat postscaler

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

void set_auto_repeat(bool enabled, int threshold);
esp_err_t rc5_receiver_init(rc5_handler_t rc5_handler);
void rc5_terminate(void);

#ifdef __cplusplus
}   // extern "C"
#endif

// end of rc5_receiver.h