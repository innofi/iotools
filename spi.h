#pragma once

// types definition
#include <stdint.h>

#include <stdlib.h>
#include <string.h>

// CLK - GPIO14
#define PIN_CLK 14
// MOSI - GPIO 13
#define PIN_MOSI 13
// MISO - GPIO
#define PIN_MISO 12
// RESET - unused
#define PIN_RESET 0
// CS - GPIO 32 
#define PIN_CS 20

uint8_t spi_xferbyte( uint8_t byte );
void spi_xfer( uint8_t cmd, uint8_t * data, size_t len) ;
void spi_init( void );