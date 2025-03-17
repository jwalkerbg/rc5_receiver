// rc5_receiver.h

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Version information
#define RC5_RECEIVER_VERSION CONFIG_RC5_RECEIVER_VERSION

// Events
typedef enum {
    RC5_EVENT_SHORT_PRESS,
    RC5_EVENT_LONG_PRESS_START,
    RC5_EVENT_LONG_PRESS_END
} rc5_event_t;

// RC5 command data structure
typedef union {
    struct {
        uint16_t command:6;
        uint16_t address:5;
        uint16_t toggle:1;
        uint16_t start:2;
        uint16_t reserved:2;
    };
    uint16_t frame;
} rc5_data_t;   // RC5 command data structure

typedef void (*rc5_handler_t)(rc5_event_t event, rc5_data_t rc5_data);

void rc5_set_event_callback(rc5_handler_t callback);
esp_err_t rc5_receiver_init(rc5_handler_t rc5_handler);
void rc5_terminate(void);
const char* rc5_event_name(rc5_event_t event);

#ifdef __cplusplus
}   // extern "C"
#endif

// end of rc5_receiver.h