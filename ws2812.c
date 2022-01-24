// Copyright 2019 Espressif Systems (Shanghai) PTE LTD
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_pm.h"
#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include <math.h>
#include "esp_attr.h"
#include "driver/rmt.h"
#include "driver/rtc_io.h"

#include "ws2812.h"

static const char *TAG = "RMT";

#define BYTES_PER_PIXEL (3)

// this is for the WS2813B-V2 version, see data sheet
#define _WS2812_T0H_NS (400)
#define _WS2812_T0L_NS (800)
#define _WS2812_T1H_NS (800)
#define _WS2812_T1L_NS (800)
#define _WS2812_RESET_US (400)
// this is for the WD2813E model
#define WS2812_T0H_NS (300)
#define WS2812_T0L_NS (800)
#define WS2812_T1H_NS (800)
#define WS2812_T1L_NS (300)
#define WS2812_RESET_US (400)

#define RMT_CHANNEL RMT_CHANNEL_0

static uint32_t ws2812_t0h_ticks = 0;
static uint32_t ws2812_t1h_ticks = 0;
static uint32_t ws2812_t0l_ticks = 0;
static uint32_t ws2812_t1l_ticks = 0;

static uint8_t* buffer = NULL;

static int ws2812_numleds = 0;
static xTaskHandle fade_task = NULL;
static esp_pm_lock_handle_t hd = NULL;

/**
 * @brief Conver RGB data to RMT format.
 *
 * @note For WS2812, R,G,B each contains 256 different choices (i.e. uint8_t)
 *
 * @param[in] src: source data, to converted to RMT format
 * @param[in] dest: place where to store the convert result
 * @param[in] src_size: size of source data
 * @param[in] wanted_num: number of RMT items that want to get
 * @param[out] translated_size: number of source data that got converted
 * @param[out] item_num: number of RMT items which are converted from source data
 */
static void IRAM_ATTR ws2812_rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
                                         size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    if (src == NULL || dest == NULL)
    {
        *translated_size = 0;
        *item_num = 0;
        return;
    }
    const rmt_item32_t bit0 = {{{ ws2812_t0h_ticks, 1, ws2812_t0l_ticks, 0 }}}; //Logical 0
    const rmt_item32_t bit1 = {{{ ws2812_t1h_ticks, 1, ws2812_t1l_ticks, 0 }}}; //Logical 1
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t *pdest = dest;
    while (size < src_size && num < wanted_num) {
        for (int i = 0; i < 8; i++) {
            // MSB first
            if (*psrc & (1 << (7 - i))) {
                pdest->val =  bit1.val;
            } else {
                pdest->val =  bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;
}


// logarithmic fading
// returns true when LED is off
static bool ws2812_fade_pixel( uint32_t index )
{
    const uint8_t divisor = 4;
    uint32_t start = index * 3;
    uint8_t fade;
    // logarithymic fade
    fade = 1 + (buffer[start + 0] / divisor);
    buffer[start + 0] = (buffer[start + 0] > fade) ? buffer[start + 0] - fade : 0;

    fade = 1 + (buffer[start + 1] / divisor);
    buffer[start + 1] = (buffer[start + 1] > fade) ? buffer[start + 1] - fade : 0;

    fade = 1 + (buffer[start + 2] / divisor);
    buffer[start + 2] = (buffer[start + 2] > fade) ? buffer[start + 2] - fade : 0;
    //ESP_LOGI(TAG, "fade %02x%02x%02x", buffer[start + 0], buffer[start + 1], buffer[start + 2]);

    if( buffer[start + 0] | buffer[start + 1] | buffer[start + 2] ){
        return true;
    } else {
        return false;
    }
}

// get the current clock frequency and set ticks
static void ws2812_set_ticks( void ){
    uint32_t counter_clk_hz = 0;
    if( rmt_get_counter_clock( RMT_CHANNEL, &counter_clk_hz ) != ESP_OK ){
        ESP_LOGE( TAG, "get rmt counter clock failed");
    }
    // ns -> ticks
    float ratio = (float)counter_clk_hz / 1e9;
    ws2812_t0h_ticks = (uint32_t)(ratio * WS2812_T0H_NS);
    ws2812_t0l_ticks = (uint32_t)(ratio * WS2812_T0L_NS);
    ws2812_t1h_ticks = (uint32_t)(ratio * WS2812_T1H_NS);
    ws2812_t1l_ticks = (uint32_t)(ratio * WS2812_T1L_NS);
    //ESP_LOGI(TAG, "ratio %f, T0H=%d", ratio, ws2812_t0h_ticks );
}


esp_err_t 
ws2812_refresh( void ) {
    esp_err_t ret = ESP_OK;

    if( hd == NULL ) return -1; // not initialized
    if( buffer == NULL ) return -1;

    esp_pm_lock_acquire(hd);
    // APB freq cannot change during transmit
    ws2812_set_ticks();
    // with wait for done
    if( rmt_write_sample( RMT_CHANNEL, buffer, ws2812_numleds * BYTES_PER_PIXEL, true ) != ESP_OK ){
        ESP_LOGE( TAG, "transmit RMT samples failed");
        return ESP_FAIL;
    }
    //ESP_LOGI(TAG, "rmt written");
    ret = rmt_wait_tx_done( RMT_CHANNEL, pdMS_TO_TICKS( 10 )); 
    // stop transmit
    rmt_tx_stop( RMT_CHANNEL ); // no loop
    // release APB lock
    esp_pm_lock_release(hd);
    return ret;
}

static void 
ws2812_clear( bool refresh ) {
    if( hd == NULL ) return; // not initialized
    if( buffer == NULL ) return;
    // Write zero to turn off all leds
    memset( buffer, 0,  ws2812_numleds * BYTES_PER_PIXEL);
    if( refresh )
        ws2812_refresh();
}




// colors of the WS2812
static uint8_t R(double T) {
  if(T >= -1.0) {
    if(T <= 0.0)
      return 255*(1);
    if(T <= 1.0)
      return 255*(-0.990*T*T*T + 2.34*T*T - 1.99*T + 0.970);
  }
  return 0;
}
static uint8_t G(double T) {
  if(T >= -1.0) {
    if(T <= 0.0)
      return 255*(-0.402*T*T*T - 0.211*T*T + 1.09*T + 0.958);
    if(T <= 1.0)
      return 255*(-0.542*T*T*T + 1.37*T*T - 1.28*T + 0.941);
  }
  return 0;
}
static uint8_t B(double T) {
  if(T >= -1.0) {
    if(T <= -0.7)
      return 255*(0);
    if(T <= 0.0)
      return 255*(0.0117*T*T*T + 2.05*T*T + 2.85*T + 1.00);
    if(T <= 1.0)
      return 255*(1);
  }
  return 0;
}


// temp in K
// bright in percent
bool
ws2812_colortemp( int index, int temp, int bright, bool refresh ){

    double f_T = ( log( temp ) - 8.79 ) / 1.8;
    double f_brite = (double)bright / 100.0;
    uint32_t start = index * BYTES_PER_PIXEL;
    if( hd == NULL ) return false; // not initialized
    if( buffer == NULL ) return false;

    uint8_t Red = R(f_T)*f_brite;
    uint8_t Grn = G(f_T)*f_brite;
    uint8_t Blu = B(f_T)*f_brite;

    //ESP_LOGI(TAG, "led %d, color %02x%02x%02x", index, Red, Grn, Blu );

    // In the order of GRB
    buffer[start + 0] = Grn;
    buffer[start + 1] = Red;
    buffer[start + 2] = Blu;

    if( refresh )
        ws2812_refresh();

    if( Red | Grn | Blu )
        return true;
    else
        return false;
}

void ws2812_color( int index, uint8_t red, uint8_t green, uint8_t blue, bool refresh ){
    uint32_t start = index * BYTES_PER_PIXEL;
    if( hd == NULL ) return; // not initialized
    if( buffer == NULL ) return; // no memory
    // In the order of GRB
    buffer[start + 0] = green;
    buffer[start + 1] = red;
    buffer[start + 2] = blue;
    if( refresh )
        ws2812_refresh();
}


/* // 30 seconds fade
def Intensity(x):
   return 2*x**2/(2700-30*x)
def fx(y):
   I = 100
   for i in range (30, 0, -1): 
       I = Intensity(i)*y
       print(I)
*/
// runs for 30 seconds, meaning we fade by 256/30
void ws2812_fadeall_task(void *pvParameter) {
    uint still_busy;
    int i;
    while( 1 ) {
        // every second do:
        vTaskDelay( 1000 / portTICK_PERIOD_MS);
        still_busy = 0;
        ESP_LOGI(TAG, "fadeing");
        for( i=1; i<ws2812_numleds; i++ ){
            /* Fade towards unlit */
            still_busy |= ws2812_fade_pixel( i );
        }
        ws2812_refresh();
        if( still_busy == 0 ){
            // end task
            fade_task = NULL;
            //ESP_LOGI(TAG, "fade done");
            vTaskDelete( NULL ); 
        }
    }
    
}

// restart possible
// starting at LED1 not zero
void ws2812_bargraph( int length ){
    int i;
    if( hd == NULL ) return; // not initialized
    if( length > ws2812_numleds )
        return;

    ws2812_clear( false );
    for( i=1; i<length; i++ ){
        ws2812_colortemp( i, 4000, 10, false ); // set LEDs
        //ESP_LOGI(TAG, "set %02x%02x%02x", buffer[(i*3) + 0], buffer[(i*3) + 1], buffer[(i*3) + 2]);
    }
    ws2812_refresh();

    if( fade_task == NULL){
        xTaskCreate(ws2812_fadeall_task, "fade_task", 3*configMINIMAL_STACK_SIZE, NULL, 1, &fade_task );
        ESP_LOGI(TAG, "fade task started");
    }
}

// -----------------  I N I T  -----------------


// Initialize on-board peripherals : led
void ws2812_init(int num_leds)
{
    if( hd != NULL )
        // ws2812 driver has already be installed
        return;
    // we need power management lock when transmitting
    esp_pm_lock_create( ESP_PM_APB_FREQ_MAX, 0, "my_rmt_apb_lock", &hd );

    gpio_pad_select_gpio( PULSE_PIN );

    // WS2812 Neopixel driver with RMT peripheral
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(PULSE_PIN, RMT_CHANNEL);
    config.clk_div = 1; // full speed, 80MHz or 12.5ns

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    ws2812_numleds = num_leds;

    // 24 bits per led
    buffer = malloc( ws2812_numleds * BYTES_PER_PIXEL );
    if( buffer == NULL ) {
        ws2812_deinit();
        ws2812_numleds = 0;
        ESP_LOGE( TAG, "request memory for ws2812 failed" );
        return;
    }
    // set ws2812 to rmt adapter
    rmt_translator_init( RMT_CHANNEL, ws2812_rmt_adapter );

    ws2812_clear( true ); // off led

    ESP_LOGI(TAG, "RMT init");

}

void ws2812_deinit( void ){
    ESP_LOGI(TAG, "deinit");
    ws2812_clear( true ); // off led
/*
    free(buffer);
    buffer = NULL;
    rmt_driver_uninstall( RMT_CHANNEL );
*/
    //rtc_gpio_isolate(rtc_io_number_get(PULSE_PIN)); crashes
}