/**
 * @file qma7981.c
 * @brief 
 * @version 0.1
 * @date 2021-09-01
 * 
 * @copyright Copyright (c) 2021
 * original: https://github.com/espressif/esp-who/tree/master/components/modules/imu
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_sleep.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#include "settings.h"
// generic I2C code
#include "i2c_bus.h"
// specific initialization
#include "i2c.h"
#include "qma7981.h"

static const char *TAG = "qma7981";
static qma_range_t qma_range = QMA_RANGE_2G;
static i2c_bus_device_handle_t qma7981_handle;

// receive interrupts from the accelerometer
QueueHandle_t accelQueue = NULL;

/**
 * @brief 
 * 
 * @param reg_addr 
 * @param data 
 * @return esp_err_t 
 */
static esp_err_t qma7981_read_byte(uint8_t reg_addr, uint8_t *data)
{
	return i2c_bus_read_byte(qma7981_handle, reg_addr, data);
}

static esp_err_t qma7981_write_byte(uint8_t reg_addr, uint8_t data)
{
	return i2c_bus_write_byte(qma7981_handle, reg_addr, data);
}

static esp_err_t qma7981_read_bytes(uint8_t reg_addr, size_t data_len, uint8_t *data)
{
	return i2c_bus_read_bytes(qma7981_handle, reg_addr, data_len, data);
}



// called quickly when wakeup from deep sleep
static void qma7981_create_interrupt_service( ACCEL_INT_CALLBACK accel_isr_handler ){
	// clear interrupts in case latch is enabled
	// .....................

	// after deep sleep, need to setup timer, interrupts and tasks again
	gpio_config_t io_conf;
	// interrupt of falling edge
    io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE; // was positive edge
    // bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL<<GPIO_INPUT_ACCEL_INT1);
    //s et as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf.pull_up_en = 1;
	io_conf.pull_down_en = 0;
    gpio_config( &io_conf );

    // accelQueue for ISR messsages
    accelQueue = xQueueCreate( 1, sizeof( uint64_t ) );

	// hook ISR handler for specific gpio pin any edge
	gpio_isr_handler_add( GPIO_INPUT_ACCEL_INT1, accel_isr_handler, (void*) GPIO_INPUT_ACCEL_INT1);

	// check if the beertap handle is still cranked, install IRQ handler then
	if( gpio_get_level( GPIO_INPUT_ACCEL_INT1 ) == 0 ){
		// waiting for close faucet interrupt
		ESP_LOGI( TAG, "start isr, tap handle LOW" ); 
		//ESP_LOGI(TAG, "allow ISR active on high, time %lldms", get_rtc_time() - start_time );
		settingsV.atag_busy.who_s_busy.accelBusy = true;
	} else {
		ESP_LOGI( TAG, "start isr, tap handle HIGH" );
		// otherwise just ignore the wakeup and allow deep sleep again
		esp_sleep_enable_ext1_wakeup( (1ULL<<GPIO_INPUT_ACCEL_INT1), ESP_EXT1_WAKEUP_ALL_LOW );
		//ESP_LOGI(TAG, "allow ACCEL wakeup on low");
		settingsV.atag_busy.who_s_busy.accelBusy = false;
	}
}


esp_err_t qma7981_init( bool reset, ACCEL_INT_CALLBACK cb )
{
	// initialize once
	if (NULL != qma7981_handle ){
		return ESP_FAIL;
	}

	esp_err_t ret_val = ESP_OK;

	qma7981_handle = i2c_bus_initialize( QMA7981_I2C_ADDR );
	assert(qma7981_handle != NULL);

	uint8_t id = 0;
	ret_val |= qma7981_read_byte( QMA7981_REG_CHIP_ID, &id );
	ESP_LOGW(TAG, "ID : %02X, should be.. ", id);

	/* ******************************** ******************************** */
	ret_val |= qma7981_write_byte(QMA7981_REG_PWR_MANAGE, 0xC0); /* Exit sleep mode*/
	vTaskDelay(pdMS_TO_TICKS(20)); // wakeup delay
	ret_val |= qma7981_write_byte(QMA7981_REG_RANGE, QMA_RANGE_2G);				  /* Set range */
	ret_val |= qma7981_write_byte(QMA7981_REG_BAND_WIDTH, QMA_BANDWIDTH_1024_HZ); /* Set bandwidth */
	/* ******************************** ******************************** */

	// configure interrupt service with callback
	qma7981_create_interrupt_service( cb );

	return ret_val;
}



esp_err_t qma7981_get_step(uint16_t *data)
{
	esp_err_t ret_val = ESP_OK;
	uint8_t step_h = 0, step_l = 0;

	if (NULL == data)
	{
		return ESP_ERR_INVALID_ARG;
	}

	ret_val |= qma7981_read_byte(0x07, &step_l);
	ret_val |= qma7981_read_byte(0x08, &step_h);

	*data = (step_h << 8) + step_l;

	return ret_val;
}

/**
 * @brief 
 * 
 * @param range 
 * @return esp_err_t 
 */
esp_err_t qma7981_set_range(qma_range_t range)
{
	esp_err_t ret_val = qma7981_write_byte(QMA7981_REG_RANGE, range);
	qma_range = range;

	return ret_val;
}

/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @param z 
 * @return esp_err_t 
 */
esp_err_t qma7981_get_acce(float *x, float *y, float *z)
{
	float multiple = 2;
	esp_err_t ret_val = ESP_OK;
	struct qma_acce_data_t
	{
		int16_t x;
		int16_t y;
		int16_t z;
	} data;

	switch (qma_range)
	{
	case QMA_RANGE_2G:
		multiple = 2;
		break;
	case QMA_RANGE_4G:
		multiple = 4;
		break;
	case QMA_RANGE_8G:
		multiple = 8;
		break;
	case QMA_RANGE_16G:
		multiple = 16;
		break;
	case QMA_RANGE_32G:
		multiple = 32;
		break;
	default:
		multiple = 2;
		break;
	}

	ret_val |= qma7981_read_bytes(QMA7981_REG_DX_L, 6, &data);

	/* QMA7981's range is 14 bit. Adjust data format */
	data.x >>= 2;
	data.y >>= 2;
	data.z >>= 2;

	/* Convert to acceleration of gravity */
	*x = data.x / (float)(1 << 13) * multiple;
	*y = data.y / (float)(1 << 13) * multiple;
	*z = data.z / (float)(1 << 13) * multiple;

	return ret_val;
}
