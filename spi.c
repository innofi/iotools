#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include <time.h>
#include <sys/time.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>


#include "spi.h"

#define MIN(x,y) (x < y ? x : y)
#define round(x) ({ \
  typeof(x) _x = (x); \
  (_x>=0) ? (long)(_x+0.5) : (long)(_x-0.5); \
})



static char TAG[] = "SPI";

static spi_device_handle_t handle_spi;      // SPI handle.

#undef PS_DEBUG

uint8_t 
spi_xferbyte( uint8_t byte ) {

	spi_transaction_t trans_desc;

	trans_desc.flags     = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
	trans_desc.length    = 8; // Number of bits NOT number of bytes.
	trans_desc.rxlength  = 0;
	trans_desc.tx_data[0] = byte;
	trans_desc.tx_data[1] = 0;
	trans_desc.rx_data[0] = 0;

	if( spi_device_transmit(handle_spi, &trans_desc) != ESP_OK ){
		ESP_LOGE(TAG, "byte transfer error");
	} else {
		//ESP_LOGI(TAG, "sent 0x%x, got 0x%x", byte, trans_desc.rx_data[0]);
	}
	return trans_desc.rx_data[0];
}



void spi_xfer(uint8_t cmd, uint8_t * data, size_t len) {
  #ifdef PS_DEBUG
  {
    printf( "SPI Write: (");
    printf( "0x%02x", cmd );
    printf( ")");
    for (size_t i = 0; i < len; i++) {
      printf( ", ");
      printf( "0x%02x", data[i] );
    }
    printf( "\n" );
  }
  #endif

  cmd = spi_xferbyte(cmd);
  for (size_t i = 0; i < len; i++) {
    data[i] = spi_xferbyte(data[i]);
  }

  #ifdef PS_DEBUG
  {
    printf( "SPI Read: ");
    if (cmd != 0) {
      printf( "XX----> ");
    }
    printf( "0x%02x", cmd);
    for (size_t i = 0; i < len; i++) {
      printf( ", ");
      printf( "0x%02x", data[i]);
    }
    printf( "\n" );
  }
  #endif
}



void spi_init( void ) {

	spi_bus_config_t bus_config;
	memset(&bus_config, 0, sizeof(spi_bus_config_t));

	bus_config.sclk_io_num   = PIN_CLK; // CLK
	bus_config.mosi_io_num   = PIN_MOSI; // MOSI
	bus_config.miso_io_num   = PIN_MISO; // unused
	bus_config.quadwp_io_num = -1; // Not used
	bus_config.quadhd_io_num = -1; // Not used
	//bus_config.max_transfer_sz = 1; // one byte at a time
	ESP_LOGI(TAG, "... Initializing bus.");
	if( spi_bus_initialize( HSPI_HOST, &bus_config, 1 ) != ESP_OK ){
		ESP_LOGE(TAG, "... error.");
	}

	spi_device_interface_config_t dev_config;
	dev_config.address_bits     = 0;
	dev_config.command_bits     = 0;
	dev_config.dummy_bits       = 0;
	dev_config.mode             = 3; // pair of (CPOL, CPHA), from STEP400 firmware
	dev_config.duty_cycle_pos   = 0;
	dev_config.cs_ena_posttrans = 4;
	dev_config.cs_ena_pretrans  = 0;
	dev_config.clock_speed_hz   = 4000000; // 4MHz
	dev_config.spics_io_num     = PIN_CS;
	dev_config.flags            = 0;
	dev_config.queue_size       = 1;
	dev_config.pre_cb           = NULL;
	dev_config.post_cb          = NULL;
	ESP_LOGI(TAG, "... Adding device bus.");
	if( spi_bus_add_device( HSPI_HOST, &dev_config, &handle_spi ) != ESP_OK ){
		ESP_LOGE(TAG, "... error.");
	}

}

