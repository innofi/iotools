#pragma once
// types definition
#include <stdint.h>

#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"

// GPIO used see touch_button.c

typedef enum {
    BUTTON_SHORT = 1,
    BUTTON_LONG = 10,       // 1 second
    BUTTON_LONGLONG = 20,   // 2 seconds
} action_type_t;

// callback function to handle button presses
typedef void (*BUTTON_FUNCTION_CALL)( int button_number, action_type_t action_type );

esp_err_t touchbuttons_init( BUTTON_FUNCTION_CALL cb );