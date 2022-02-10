#pragma once
// types definition
#include <stdint.h>

#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"

#define BUTTON_DOWN (1)
#define BUTTON_UP (2)
#define BUTTON_HELD (3)

#define PIN_BIT(x) (1ULL<<x)

#define GPIO_INPUT_BUTTON_LEFT      GPIO_NUM_27
#define GPIO_INPUT_BUTTON_RIGHT     GPIO_NUM_26
#define GPIO_INPUT_BUTTON_ROTARY    GPIO_NUM_34
#define GPIO_INPUT_FOOTPEDAL        GPIO_NUM_36



typedef enum {
    BUTTON_SHORT = 1,
    BUTTON_LONG = 10,       // 1 second
    BUTTON_LONGLONG = 20,   // 2 seconds
} action_type_t;

// callback function to handle button presses
typedef void (*BUTTON_FUNCTION_CALL)( int button_number, action_type_t action_type );

void button_poll_init( unsigned long long pin_select, gpio_pull_mode_t pull_mode, 
                bool auto_mode, BUTTON_FUNCTION_CALL cb );

// function to check gpio levels, call every 10 ms
void button_check_state( void );