#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
 
#include "settings.h"
#include "button_pull.h"

#define TAG "BUTTOND"

typedef struct {
    uint8_t pin;
    uint8_t event;
} button_event_t;

typedef struct {
  uint8_t pin;
  bool inverted;
  uint16_t history;
  uint32_t down_time;
  uint32_t next_long_time;
} debounce_t;

int pin_count = -1;
debounce_t* debounce;
QueueHandle_t button_events;

// call function for ADC readings
BUTTON_FUNCTION_CALL button_pressed_handler = NULL;	

static void update_button(debounce_t *d) {
    d->history = (d->history << 1) | gpio_get_level(d->pin);
}

#define MASK   0b1111000000111111
static bool button_rose(debounce_t *d) {
    if ((d->history & MASK) == 0b0000000000111111) {
        d->history = 0xffff;
        return 1;
    }
    return 0;
}
static bool button_fell(debounce_t *d) {
    if ((d->history & MASK) == 0b1111000000000000) {
        d->history = 0x0000;
        return 1;
    }
    return 0;
}
static bool button_down(debounce_t *d) {
    if (d->inverted) return button_fell(d);
    return button_rose(d);
}
static bool button_up(debounce_t *d) {
    if (d->inverted) return button_rose(d);
    return button_fell(d);
}

#define LONG_PRESS_DURATION (2000)
#define LONG_PRESS_REPEAT (50)

static uint32_t millis() {
    return esp_timer_get_time() / 1000;
}

static void send_event(debounce_t db, int ev) {
    button_event_t event = {
        .pin = db.pin,
        .event = ev,
    };
    xQueueSend( button_events, &event, portMAX_DELAY);
}


// check buttons, designed to be called every 10 ms
void
button_check_state( void ) {
    for (int idx=0; idx<pin_count; idx++) {
        update_button(&debounce[idx]);
/*
        if (debounce[idx].down_time && millis() >= debounce[idx].next_long_time) {
            ESP_LOGI(TAG, "%d LONG", debounce[idx].pin);
            debounce[idx].next_long_time = debounce[idx].next_long_time + LONG_PRESS_REPEAT;
            send_event(debounce[idx], BUTTON_HELD);

        } else 
*/        
        if (button_down(&debounce[idx]) && debounce[idx].down_time == 0) {
            debounce[idx].down_time = millis();
            //ESP_LOGI(TAG, "%d DOWN", debounce[idx].pin);
            debounce[idx].next_long_time = debounce[idx].down_time + LONG_PRESS_DURATION;
            send_event(debounce[idx], BUTTON_DOWN);

        } else if (button_up(&debounce[idx])) {
            debounce[idx].down_time = 0;
            //ESP_LOGI(TAG, "%d UP", debounce[idx].pin);
            send_event(debounce[idx], BUTTON_UP);
        }
    }
}

// task checks buttons every 10 ms
static void button_sub_task(void *pvParameter)
{
    while( true ) {
        button_check_state();
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

static void button_main_task(void *pvParameter){
    button_event_t ev;
    // process queue
    while (true) {
        if( xQueueReceive( button_events, &ev, portMAX_DELAY )) {
            // ignore DOWN events
            //if ( ev.event == BUTTON_UP ) {
                if( button_pressed_handler ){
                    button_pressed_handler( ev.pin, ev.event ); // second is the event type
                }
            //}
        }
    }
}


// init GPIOs and start the button tasks
// no interrupts for GPIO
void
button_poll_init( unsigned long long pin_select, gpio_pull_mode_t pull_mode, bool auto_mode, BUTTON_FUNCTION_CALL cb ) {

    button_event_t ev;

    if (pin_count != -1) {
        ESP_LOGI(TAG, "Already initialized");
        return;
    }

    // Configure the pins
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;    
    io_conf.pull_up_en = (pull_mode == GPIO_PULLUP_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN);
    io_conf.pull_down_en = (pull_mode == GPIO_PULLDOWN_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN);
    io_conf.pin_bit_mask = pin_select;
    gpio_config(&io_conf);

    // Scan the pin map to determine number of pins
    pin_count = 0;
    for (int pin=0; pin<=39; pin++) {
        if( PIN_BIT(pin) & pin_select ) {
            pin_count++;
        }
    }

    // Initialize global state and queue
    debounce = calloc( pin_count, sizeof( debounce_t ));
    button_events = xQueueCreate( 4, sizeof( button_event_t ));

    // Scan the pin map to determine each pin number, populate the state
    uint32_t idx = 0;
    for (int pin=0; pin<=39; pin++) {
        if( PIN_BIT(pin) & pin_select ) {
            ESP_LOGI(TAG, "Registering button input: %d", pin);
            debounce[idx].pin = pin;
            debounce[idx].down_time = 0;
            debounce[idx].inverted = true;
            if (debounce[idx].inverted) debounce[idx].history = 0xffff;
            idx++;
        }
    }

    // init callback function (global)
    if( cb != NULL ){
        button_pressed_handler = cb;

        // Spawn a task to monitor the pins and send events to the queue
        if( auto_mode ) {
            xTaskCreate( &button_sub_task, "button_sub_task", 4096, NULL, 10, NULL );
        }
        // task checking for events in the queue and callback
        xTaskCreate( &button_main_task, "button_main_task", 4096, NULL, 10, NULL );
        ESP_LOGI(TAG, "buttons initialized!" );

        // allow deep sleep wakeup with button
        esp_sleep_enable_ext0_wakeup( GPIO_INPUT_BUTTON_ROTARY, 0 ); // wakeup on low
        // keep GPIO pin on
        esp_sleep_pd_config( ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON );

    } else {
        ESP_LOGI(TAG, "buttons NOT initialized!" );
    }
}





