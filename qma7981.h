/**
 * @file qma7981.h
 * @brief 
 * @version 0.1
 * @date 2021-09-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// interrupt gpio
#define GPIO_INPUT_ACCEL_INT1 (GPIO_NUM_26)
// interrupt callback
typedef void (*ACCEL_INT_CALLBACK)( uint64_t time_ms );
// receive interrupts from the accelerometer
extern QueueHandle_t accelQueue;

// chip's address (lower)
#define QMA7981_I2C_ADDR (0x12)
// registers
#define QMA7981_REG_CHIP_ID 0x00
#define QMA7981_REG_DX_L 0x01
#define QMA7981_REG_DX_H 0x02
#define QMA7981_REG_DY_L 0x03
#define QMA7981_REG_DY_H 0x04
#define QMA7981_REG_DZ_L 0x05
#define QMA7981_REG_DZ_H 0x06
#define QMA7981_REG_STEP_L 0x07
#define QMA7981_REG_STEP_H 0x08
#define QMA7981_REG_INT_STAT_0 0x0A
#define QMA7981_REG_INT_STAT_1 0x0B
#define QMA7981_REG_INT_STAT_4 0x0D
#define QMA7981_REG_RANGE 0x0F
#define QMA7981_REG_BAND_WIDTH 0x10
#define QMA7981_REG_PWR_MANAGE 0x11
#define QMA7981_REG_STEP_CONF_0 0x12
#define QMA7981_REG_STEP_CONF_1 0x13
#define QMA7981_REG_STEP_CONF_2 0x14
#define QMA7981_REG_STEP_CONF_3 0x15
#define QMA7981_REG_INT_EN_0 0x16
#define QMA7981_REG_INT_EN_1 0x17
#define QMA7981_REG_INT_MAP_0 0x19
#define QMA7981_REG_INT_MAP_1 0x1A
#define QMA7981_REG_INT_MAP_2 0x1B
#define QMA7981_REG_INT_MAP_3 0x1C
#define QMA7981_REG_SIG_STEP_TH 0x1D
#define QMA7981_REG_STEP 0x1F

typedef enum {
	QMA_RANGE_2G = 0b0001,
	QMA_RANGE_4G = 0b0010,
	QMA_RANGE_8G = 0b0100,
	QMA_RANGE_16G = 0b1000,
	QMA_RANGE_32G = 0b1111,
} qma_range_t;	/* Others will be 2G */

typedef enum {
	QMA_BANDWIDTH_128_HZ = 0b111,
	QMA_BANDWIDTH_256_HZ = 0b110,
	QMA_BANDWIDTH_1024_HZ = 0b101,
} qma_bandwidth_t;

/**
 * @brief 
 * 
 * @return esp_err_t 
 */
esp_err_t qma7981_init( bool reset, ACCEL_INT_CALLBACK cb );

/**
 * @brief 
 * 
 * @param range 
 * @return esp_err_t 
 */
esp_err_t qma7981_set_range(qma_range_t range);

/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @param z 
 * @return esp_err_t 
 */
esp_err_t qma7981_get_acce(float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif
