
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "settings.h"
#include "button_irq.h"

#define TAG "BUTTON"
#define DEBOUNCE   40000      // debounce time
#define LONGPRESS   210000  // units of long press


// call function for ADC readings
BUTTON_FUNCTION_CALL button_pressed_handler = NULL;	

static xQueueHandle gpio_evt_queue = NULL;
static int64_t time_got_interrupt = 0L;
static gpio_num_t input_button;

// timer
static void reinstall_isr_timer_cb(void* arg);
static esp_timer_handle_t reinstall_isr_timer;
const esp_timer_create_args_t reinstall_isr_timer_args = {
    .callback = &reinstall_isr_timer_cb,
    .arg = NULL,
};

// IRQ
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR( gpio_evt_queue, &gpio_num, NULL );
}

// called after debounce time
static void reinstall_isr_timer_cb( void* arg ){
    // hook isr handler back to button
    gpio_isr_handler_add( input_button, gpio_isr_handler, (void*)input_button );
}

// task handling buttons
static void button_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            //ESP_LOGI(TAG, "GPIO[%d] intr, val: %d", io_num, gpio_get_level(io_num));

            if( gpio_get_level(io_num) == 0 ){
                // stop interrupts for debounce
                gpio_isr_handler_remove( io_num );
                esp_timer_stop( reinstall_isr_timer) ; // just in case
                ESP_ERROR_CHECK( esp_timer_start_once( reinstall_isr_timer, DEBOUNCE )); // 40ms

                //ESP_LOGI( TAG, "button %d activated!", io_num );
                //button pressed, don't go into deep sleep
                time_got_interrupt =  esp_timer_get_time();
                //printf("init ");
            } else {
                // button released, call back function
                if( time_got_interrupt != 0 ) {
                    // spurious interrupt without button press occure, noise?

                    int64_t time_touched = (esp_timer_get_time() - time_got_interrupt) / LONGPRESS ; 
                    if( time_touched > 40 ) time_touched = 2; // could be an error, ignore when press is too long
                    //ESP_LOGI(TAG, "button %d active for %lld units", io_num, time_touched ); // in units of 210ms
                    if( button_pressed_handler ){
                        button_pressed_handler( io_num, time_touched );
                    }
                    // no longer busy
                    time_got_interrupt = 0;
                }
            }
        }
    }
}



esp_err_t button_irq_init( BUTTON_FUNCTION_CALL cb ){
    gpio_config_t io_conf;
    int usb_enabled = 0;

    gpio_set_direction( GPIO_INPUT_USB_DETECT, GPIO_MODE_INPUT );
    gpio_set_pull_mode( GPIO_INPUT_USB_DETECT, GPIO_PULLDOWN_ONLY );
    usb_enabled = gpio_get_level( GPIO_INPUT_USB_DETECT ); // check if USB is connected
    // set to floating for power consumption
    gpio_set_pull_mode( GPIO_INPUT_USB_DETECT, GPIO_FLOATING );
 

    if( usb_enabled ){
        // it is HostER hardware
        input_button = GPIO_INPUT_BUTTON_HOSTER;
    } else {
        //bit mask of the pins, use GPIO13 here
        input_button = GPIO_INPUT_BUTTON_MINIE4;
    }

    //interrupt of any edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;    
    io_conf.pin_bit_mask = (1ULL<<input_button);        
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    // MUST have external pull up resistor when using deep sleep (!)
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate( 10, sizeof(uint32_t));
    // start gpio task
    xTaskCreate(button_task, "button_task", 4*1024, NULL, 5, NULL);

    // hook isr handler for specific gpio pin
    gpio_isr_handler_add( input_button, gpio_isr_handler, (void*)input_button );
    ESP_LOGI(TAG, "Mini button %d initialized!", input_button );


    // init callback function
    if( cb != NULL )
        button_pressed_handler = cb;

    // setup timer for debounce
	ESP_ERROR_CHECK(esp_timer_create( &reinstall_isr_timer_args, &reinstall_isr_timer));
    // The timer has been created but is not running yet 

    // allow deep sleep wakeup with button
    esp_sleep_enable_ext0_wakeup( input_button, 0 ); // wakeup on low
    // keep GPIO pin on
    esp_sleep_pd_config( ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    return ESP_OK;
}