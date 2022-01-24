// Copyright 2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "hal/pcnt_hal.h"
#include "driver/pcnt.h"
#include "rotary.h"

static const char *TAG = "ROTARY";

// manual rotary encoder value implementation, due to bug
#define ROTARY_MAX_VALUE    (10000)
#define ROTARY_MIN_VALUE    (0)
int16_t rotary_value = 0;

// global
rotary_encoder_t *rotary_encoder = NULL;

#define ROTARY_CHECK(a, msg, tag, ret, ...)                                       \
    do {                                                                          \
        if (unlikely(!(a))) {                                                     \
            ESP_LOGE(TAG, "%s(%d): " msg, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret_code = ret;                                                       \
            goto tag;                                                             \
        }                                                                         \
    } while (0)

#define EC11_PCNT_DEFAULT_HIGH_LIMIT (1)
#define EC11_PCNT_DEFAULT_LOW_LIMIT  (-1)

// A flag to identify if pcnt isr service has been installed.
static bool is_pcnt_isr_service_installed = false;
// A lock to avoid pcnt isr service being installed twice in multiple threads.
static _lock_t isr_service_install_lock;
#define LOCK_ACQUIRE() _lock_acquire(&isr_service_install_lock)
#define LOCK_RELEASE() _lock_release(&isr_service_install_lock)

typedef struct {
    int accumu_count;
    rotary_encoder_t parent;
    pcnt_unit_t pcnt_unit;
} ec11_t;

static esp_err_t ec11_set_glitch_filter(rotary_encoder_t *encoder, uint32_t max_glitch_us)
{
    esp_err_t ret_code = ESP_OK;
    ec11_t *ec11 = __containerof(encoder, ec11_t, parent);

    /* Configure and enable the input filter */
    ROTARY_CHECK(pcnt_set_filter_value(ec11->pcnt_unit, max_glitch_us * 80) == ESP_OK, "set glitch filter failed", err, ESP_FAIL);

    if (max_glitch_us) {
        pcnt_filter_enable(ec11->pcnt_unit);
    } else {
        pcnt_filter_disable(ec11->pcnt_unit);
    }

    return ESP_OK;
err:
    return ret_code;
}

static esp_err_t ec11_start(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = __containerof(encoder, ec11_t, parent);
    pcnt_counter_resume(ec11->pcnt_unit);
    return ESP_OK;
}

static esp_err_t ec11_stop(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = __containerof(encoder, ec11_t, parent);
    pcnt_counter_pause(ec11->pcnt_unit);
    return ESP_OK;
}

static int ec11_get_counter_value(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = __containerof(encoder, ec11_t, parent);
    int16_t val = 0;
    pcnt_get_counter_value(ec11->pcnt_unit, &val);
    return val + ec11->accumu_count;
}

static esp_err_t ec11_del(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = __containerof(encoder, ec11_t, parent);
    free(ec11);
    return ESP_OK;
}

static void __attribute__((__unused__)) ec11_pcnt_overflow_handler(void *arg)
{
    ec11_t *ec11 = (ec11_t *)arg;
    uint32_t status = 0;

    // ignore interrupt when a button is pressed
    //if( settingsV.button_pressed ) return;

    pcnt_get_event_status(ec11->pcnt_unit, &status);
    
    static RTC_RODATA_ATTR const char fmt_str[] = "rotary limit %c c:%d\n";
    if( status & PCNT_EVT_H_LIM ) {
        //ec11->accumu_count = EC11_PCNT_DEFAULT_HIGH_LIMIT-1;
        //ets_printf(fmt_str, '+', val );
        rotary_value++;
    } else if( status & PCNT_EVT_L_LIM ) {
        //ec11->accumu_count += EC11_PCNT_DEFAULT_LOW_LIMIT;
        //ets_printf(fmt_str, '-', val );
        rotary_value--;
    } else {
        ets_printf(fmt_str, '=', 0 );
    }
}

esp_err_t rotary_encoder_new_ec11(const rotary_encoder_config_t *config, rotary_encoder_t **ret_encoder)
{
    esp_err_t ret_code = ESP_OK;
    ec11_t *ec11 = NULL;

    ROTARY_CHECK(config, "configuration can't be null", err, ESP_ERR_INVALID_ARG);
    ROTARY_CHECK(ret_encoder, "can't assign context to null", err, ESP_ERR_INVALID_ARG);

    ec11 = calloc(1, sizeof(ec11_t));
    ROTARY_CHECK(ec11, "allocate context memory failed", err, ESP_ERR_NO_MEM);

    ec11->pcnt_unit = (pcnt_unit_t)(config->dev);

    // Configure channel 0
    pcnt_config_t dev_config = {
        .pulse_gpio_num = config->phase_a_gpio_num,
        .ctrl_gpio_num = config->phase_b_gpio_num,
        .channel = PCNT_CHANNEL_0,
        .unit = ec11->pcnt_unit,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_DEC,   // Count down on the positive edge
        .neg_mode = PCNT_COUNT_INC,
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
                // Set the maximum and minimum limit values to watch
        .counter_h_lim = EC11_PCNT_DEFAULT_HIGH_LIMIT,
        .counter_l_lim = EC11_PCNT_DEFAULT_LOW_LIMIT,
    };
    ROTARY_CHECK(pcnt_unit_config(&dev_config) == ESP_OK, "config pcnt channel 0 failed", err, ESP_FAIL);

    // Configure channel 1
    dev_config.pulse_gpio_num = config->phase_b_gpio_num;
    dev_config.ctrl_gpio_num = config->phase_a_gpio_num;
    dev_config.channel = PCNT_CHANNEL_1;   
    dev_config.pos_mode = PCNT_COUNT_INC;   // The other way!
    dev_config.neg_mode = PCNT_COUNT_DEC;
/*            // Set the maximum and minimum limit values to watch
    dev_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting direction if low
    dev_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
    dev_config.counter_h_lim = EC11_PCNT_DEFAULT_HIGH_LIMIT;
    dev_config.counter_l_lim = EC11_PCNT_DEFAULT_HIGH_LIMIT;
*/    
    ROTARY_CHECK(pcnt_unit_config(&dev_config) == ESP_OK, "config pcnt channel 1 failed", err, ESP_FAIL);

    // PCNT pause and reset value
    pcnt_counter_pause(ec11->pcnt_unit);
    pcnt_counter_clear(ec11->pcnt_unit);


    // register interrupt handler in a thread-safe way
    LOCK_ACQUIRE();
    if (!is_pcnt_isr_service_installed) {
        ROTARY_CHECK(pcnt_isr_service_install(0) == ESP_OK, "install isr service failed", err, ESP_FAIL);
        // make sure pcnt isr service won't be installed more than one time
        is_pcnt_isr_service_installed = true;
    }
    LOCK_RELEASE();

    pcnt_isr_handler_add( ec11->pcnt_unit, ec11_pcnt_overflow_handler, ec11 );

    //pcnt_set_event_value( ec11->pcnt_unit, PCNT_EVT_THRES_0, -1 );  // when arrive to minimum threshold trigger
    //pcnt_set_event_value( ec11->pcnt_unit, PCNT_EVT_THRES_1, 101 );  // when arrive to max threshold trigger

    pcnt_event_enable( ec11->pcnt_unit, PCNT_EVT_L_LIM );
    pcnt_event_enable( ec11->pcnt_unit, PCNT_EVT_H_LIM );
    //pcnt_event_enable( ec11->pcnt_unit, PCNT_EVT_THRES_0 );
    //pcnt_event_enable( ec11->pcnt_unit, PCNT_EVT_THRES_1 );
    //pcnt_event_enable( ec11->pcnt_unit, PCNT_EVT_ZERO );

    ec11->parent.del = ec11_del;
    ec11->parent.start = ec11_start;
    ec11->parent.stop = ec11_stop;
    ec11->parent.set_glitch_filter = ec11_set_glitch_filter;
    ec11->parent.get_counter_value = ec11_get_counter_value;

    *ret_encoder = &(ec11->parent);
    return ESP_OK;
err:
    if (ec11) {
        free(ec11);
    }
    return ret_code;
}

void 
rotary_encoder_init( void ){
    // Rotary encoder underlying device is represented by a PCNT unit in this example
    uint32_t pcnt_unit = 0;
    rotary_value = 0;
    // Create rotary encoder instance (gobal var)
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, 
                                            GPIO_INPUT_CHAN_A, 
                                            GPIO_INPUT_CHAN_B);

    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &rotary_encoder));
    // Filter out glitch (1us)
    ESP_ERROR_CHECK(rotary_encoder->set_glitch_filter( rotary_encoder, 10 )); // tried with 10us
    // Start encoder
    ESP_ERROR_CHECK(rotary_encoder->start(rotary_encoder));
    // check
    ESP_LOGI(TAG, "Encoder value: %d", rotary_encoder->get_counter_value(rotary_encoder));

}

void rotary_encoder_task_ec11(void* ignore)
{
    rotary_encoder_init();

    // Report counter value
    while (1) {
        ESP_LOGI(TAG, "Encoder value: %d", rotary_encoder->get_counter_value(rotary_encoder));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

bool
rotary_encode_get_value( int *current ){
    static int16_t last_value = 0;

    // limit the values 
    if( rotary_value > ROTARY_MAX_VALUE ) rotary_value = ROTARY_MAX_VALUE;
    if( rotary_value <= ROTARY_MIN_VALUE ) rotary_value = ROTARY_MIN_VALUE;

    if( rotary_value != last_value ){
        last_value = rotary_value;
        // send data out
        if( current != NULL ){
            *current = rotary_value;
        }
        return true;
    } else {
        return false; // no change
    }

}

void
rotary_encode_set_value( int new_value ){
    rotary_value = new_value;
}