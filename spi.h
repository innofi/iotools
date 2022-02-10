#pragma once

// types definition
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"

#if CONFIG_MINI_PRODUCT == 3
// BnW powerstep01 SPI pins
// CLK - GPIO14
#   define PIN_CLK 14
// MOSI - GPIO 13
#   define PIN_MOSI 13
// MISO - GPIO
#   define PIN_MISO -1
// RESET - unused
#   define PIN_RESET -1
// CS - GPIO 32 
#   define PIN_CS 32

#elif CONFIG_MINI_PRODUCT == 4
// BeerPULL ePaper display pins
// CLK - GPIO14
#   define PIN_CLK 14
// MOSI - GPIO 13
#   define PIN_MOSI 12
// MISO - GPIO
#   define PIN_MISO -1
// RESET - unused
#   define PIN_RESET -1
// CS - GPIO 32 
#   define PIN_CS 8

#elif CONFIG_MINI_PRODUCT == 2
// IOLink SPI pins
// CLK - GPIO14
#   define PIN_CLK 14
// MOSI - GPIO 13
#   define PIN_MOSI 13
// MISO - GPIO
#   define PIN_MISO 12
// RESET - unused
#   define PIN_RESET -1
// CS - GPIO 32 
#   define PIN_CS -1

#else 
// DEFAUT
// CLK - GPIO14
#   define PIN_CLK -1
// MOSI - GPIO 13
#   define PIN_MOSI -1
// MISO - GPIO
#   define PIN_MISO -1
// RESET - unused
#   define PIN_RESET -1
// CS - GPIO 32 
#   define PIN_CS -1
#endif 

uint8_t spi_xferbyte( uint8_t byte );
void spi_xfer( uint8_t cmd, uint8_t * data, size_t len) ;
void spi_init( void );