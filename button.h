#pragma once
// types definition
#include <stdint.h>

#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"

// button is on GPIO13 RTC_GPIO14
#define GPIO_INPUT_BUTTON_MINIE4     GPIO_NUM_13
// HostER hardware 
#define GPIO_INPUT_BUTTON_HOSTER     GPIO_NUM_27

#define GPIO_INPUT_USB_DETECT        GPIO_NUM_4

typedef enum {
    BUTTON_SHORT = 1,
    BUTTON_LONG = 10,       // 1 second
    BUTTON_LONGLONG = 20,   // 2 seconds
} action_type_t;

// callback function to handle button presses
typedef void (*BUTTON_FUNCTION_CALL)( int button_number, action_type_t action_type );

esp_err_t button_init( BUTTON_FUNCTION_CALL cb );