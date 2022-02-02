/* Touch Pad Interrupt 

   https://github.com/espressif/esp-iot-solution/blob/master/documents/evaluation_boards/esp32_sense_kit_guide_en.md
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/uart.h"

#include "driver/touch_pad.h"
#include "soc/rtc_periph.h"
#include "soc/sens_periph.h"

#include "touch_button.h"

static const char* TAG = "TOUCH";

// call function for ADC readings
BUTTON_FUNCTION_CALL button_pressed_handler = NULL;	

#define LONGPRESS   210000      // resolution in us long press time

#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_THRESH_PERCENT  (80)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

static bool release_timer_running = false;
static void touchrelease_timer_callback(void* arg);
static esp_timer_handle_t touchrelease_timer;
static int64_t time_got_interrupt;
static int touchbutton = 0;
const esp_timer_create_args_t touchrelease_timer_args = {
    .callback = &touchrelease_timer_callback,
    .arg = (void*)&touchbutton,
};



// atags: 7, 8 and 9
static bool s_pad_activated[TOUCH_PAD_MAX];
// hardware ATAG Simple 0
// using just one pad
static bool activate_button[TOUCH_PAD_MAX] = {
    false,  // = 0/*!< Touch pad channel 0 is GPIO4(ESP32) */
    false,  /*!< Touch pad channel 1 is GPIO0(ESP32) / GPIO1(ESP32-S2) */
    true,  /*!< Touch pad channel 2 is GPIO2(ESP32) / GPIO2(ESP32-S2) */
    true,  /*!< Touch pad channel 3 is GPIO15(ESP32) / GPIO3(ESP32-S2) */
    true,  /*!< Touch pad channel 4 is GPIO13(ESP32) / GPIO4(ESP32-S2) */
    false,  /*!< Touch pad channel 5 is GPIO12(ESP32) / GPIO5(ESP32-S2) */
    false,  /*!< Touch pad channel 6 is GPIO14(ESP32) / GPIO6(ESP32-S2) */
    false,  /*!< Touch pad channel 7 is GPIO27(ESP32) / GPIO7(ESP32-S2) */
    false,  /*!< Touch pad channel 8 is GPIO33(ESP32) / GPIO8(ESP32-S2) */ // works but not as good
    false,  /*!< Touch pad channel 9 is GPIO32(ESP32) / GPIO9(ESP32-S2) */ // not working
};

// all the same
static uint32_t s_pad_init_val[TOUCH_PAD_MAX];

/*
  Read values sensed at all available touch pads.
  Use 2 / 3 of read value as the threshold
  to trigger interrupt when the pad is touched.
  Note: this routine demonstrates a simple way
  to configure activation threshold for the touch pads.
  Do not touch any pads when this routine
  is running (on application start).
 */
static void tp_set_thresholds(void)
{
    uint16_t touch_value;
    for (int i = 0; i<TOUCH_PAD_MAX; i++) {
        if( activate_button[i] ){
            //read filtered value
            touch_pad_read_filtered(i, &touch_value);
            s_pad_init_val[i] = touch_value;
            ESP_LOGI(TAG, "test init: touch pad [%d] val is %d", i, touch_value);
            //set interrupt threshold.
            ESP_ERROR_CHECK(touch_pad_set_thresh(i, touch_value * 14 / 15));
        }
    }
}


// release timer 
static void touchrelease_timer_callback(void* arg)
{
    int touched = *(int*)arg;
    // Clear information on that pad activation
    s_pad_activated[ touched ] = false;

	release_timer_running = false;
    int64_t time_touched = (esp_timer_get_time() - time_got_interrupt) / LONGPRESS ; 
    ESP_LOGI(TAG, "pad %d active for %lld units", touched, time_touched ); // in units of 210ms
    if( button_pressed_handler ){
        button_pressed_handler( touched, time_touched );
    }
}

/*
  Check if any of touch pads has been activated
  by reading a table updated by rtc_intr()
  Clear related entry in the table afterwards
  In interrupt mode, the table is updated in touch ISR.
 */
static void tp_read_task(void *pvParameter)
{
    uint16_t touch_value;

    while (1) {
        //interrupt mode, enable touch interrupt
        touch_pad_intr_enable();
        for (int i = 0; i < TOUCH_PAD_MAX; i++) {
            if (s_pad_activated[i] == true) {
                //ESP_LOGI(TAG, "T%d activated!", i);
                touchbutton = i; // global argument for call back timer routine
                if( !release_timer_running ){
                    release_timer_running = true;
                    esp_timer_stop(touchrelease_timer); // stop it even if already stopped (race condition)
                    ESP_ERROR_CHECK(esp_timer_start_once(touchrelease_timer, LONGPRESS )); // 210ms
                    time_got_interrupt =  esp_timer_get_time();
                    //printf("init ");
                } else {
                    //printf("restart 0x%lld", esp_timer_get_time() - time_got_interrupt);
                    // interrupts are coming at 200ms with additional 100us delay
                    // To restart the timer which is running, need to stop it first
                    esp_timer_stop(touchrelease_timer);
                    ESP_ERROR_CHECK(esp_timer_start_once(touchrelease_timer, LONGPRESS )); // 210ms
                }
                s_pad_activated[i] = false;
            }
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

/*
  Handle an interrupt triggered when a pad is touched.
  Recognize what pad has been touched and save it in a table.
 */
static void tp_rtc_intr(void * arg)
{
    uint32_t pad_intr = touch_pad_get_status();
    //clear interrupt
    touch_pad_clear_status();
    for (int i = 0; i < TOUCH_PAD_MAX; i++) {
        if ((pad_intr >> i) & 0x01) {
            s_pad_activated[i] = true;
        }
    }
}

/*
 * Before reading touch pad, we need to initialize the RTC IO.
 */
static void tp_touch_pad_init(void)
{
    for (int i = 0;i< TOUCH_PAD_MAX;i++) {
        //init RTC IO and mode for touch pad.
        if( activate_button[i] ){
            touch_pad_config(i, TOUCH_THRESH_NO_USE);
        }
    }
}

// https://github.com/espressif/esp-iot-solution/blob/master/documents/touch_pad_solution/touch_sensor_design_en.md
esp_err_t touchbuttons_init( BUTTON_FUNCTION_CALL cb ){
    // Initialize touch pad peripheral, it will start a timer to run a filter
    ESP_LOGI(TAG, "Initializing touch pad");
    touch_pad_init();

    // If use interrupt trigger mode, should set touch sensor FSM mode at 'TOUCH_FSM_MODE_TIMER'.
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    // setup wakeup
    touch_pad_set_group_mask(TOUCH_PAD_BIT_MASK_MAX, TOUCH_PAD_BIT_MASK_MAX, TOUCH_PAD_BIT_MASK_MAX);
    touch_pad_set_trigger_source(TOUCH_TRIGGER_SOURCE_SET1);
    // Set reference voltage for charging/discharging
    // For most usage scenarios, we recommend using the following combination:
    // the high reference valtage will be 2.7V - 1V = 1.7V, The low reference voltage will be 0.5V.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    // Init touch pad IO
    tp_touch_pad_init();
    // Initialize and start a software filter to detect slight change of capacitance.
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
    // Set thresh hold
    tp_set_thresholds();
    // Register touch interrupt ISR
    touch_pad_isr_register(tp_rtc_intr, NULL);

	// setup timer for button release 
	ESP_ERROR_CHECK(esp_timer_create(&touchrelease_timer_args, &touchrelease_timer));
    // The timer has been created but is not running yet 

    // init callback function
    if( cb != NULL )
        button_pressed_handler = cb;

    // wakeup configuration for touch buttons
    esp_sleep_enable_touchpad_wakeup();

    // Start a task to show what pads have been touched
    xTaskCreate(&tp_read_task, "touch_pad_read_task", 2*1024, NULL, 5, NULL);
    ESP_LOGI(TAG, "Touch inited");

    return ESP_OK;
}
