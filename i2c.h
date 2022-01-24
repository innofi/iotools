#pragma once

// types definition
#include <stdint.h>

#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"

esp_err_t i2cWriteRegister(uint8_t addr, uint8_t reg, uint8_t value);
esp_err_t i2cWriteRegisters(int addr, uint8_t reg, size_t size, uint8_t *data_wr);
esp_err_t i2cReadRegisters(int chip_addr, uint8_t reg, size_t size, uint8_t *data_rd);

esp_err_t i2cInitialize( void );
void i2cDisable( void );

// test
int do_i2cdump(int chip_addr );
int do_i2cdetect_cmd( bool reset );
