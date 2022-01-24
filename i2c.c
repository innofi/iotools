#include <stdio.h>

#include "sdkconfig.h"
#include <time.h>
#include <sys/time.h>

#include "driver/adc.h"
#include "driver/dac.h"
#include "esp_system.h"
#include "driver/i2c.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "soc/rtc_cntl_reg.h"

#include "i2c.h"


// I2C bit banging
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

static gpio_num_t i2c_gpio_sda = GPIO_NUM_0;
static gpio_num_t i2c_gpio_scl = GPIO_NUM_4;

static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;

/*
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
esp_err_t i2cReadRegisters(int chip_addr, uint8_t reg, size_t size, uint8_t* data_rd)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, chip_addr<<1 | WRITE_BIT, ACK_CHECK_EN);
    // register number
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

    // restart
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr<<1 | READ_BIT, ACK_CHECK_EN);

    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    // last byte with NACK
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 100 / portTICK_RATE_MS); // not longer than 300ms default watchdog timeout
    i2c_cmd_link_delete(cmd);
    return ret;
}


/*
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
esp_err_t 
i2cWriteRegisters(int addr, uint8_t reg, size_t size, uint8_t *data_wr )
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr<<1 | WRITE_BIT, ACK_CHECK_EN);
#if 0
    //i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
#else
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    for (int i = 0; i < size; i++) {
        i2c_master_write_byte(cmd, data_wr[i], ACK_CHECK_EN);
    }
#endif
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t 
i2cWriteRegister(uint8_t addr, uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr<<1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


int 
do_i2cdetect_cmd( bool reset ) {
    uint8_t address;
    if( !reset ) return 0; // deep sleep wakeup

    printf("\r\nI2C Scan\r\n");
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }
    return 0;
}


int 
do_i2cdump(int chip_addr ) {
    int size = 1;

    uint8_t data_addr;
    uint8_t data[4];
    int32_t block[16];
    printf("I2C address; 0x%X\n", chip_addr );
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f"
           "    0123456789abcdef\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j += size) {
            fflush(stdout);
            data_addr = i + j;
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, chip_addr << 1 | READ_BIT, ACK_CHECK_EN);
            if (size > 1) {
                i2c_master_read(cmd, data, size - 1, ACK_VAL);
            }
            i2c_master_read_byte(cmd, data + size - 1, NACK_VAL);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK) {
                for (int k = 0; k < size; k++) {
                    printf("%02x ", data[k]);
                    block[j + k] = data[k];
                }
            } else {
                for (int k = 0; k < size; k++) {
                    printf("XX ");
                    block[j + k] = -1;
                }
            }
        }
        printf("   ");
        for (int k = 0; k < 16; k++) {
            if (block[k] < 0) {
                printf("X");
            }
            if ((block[k] & 0xff) == 0x00 || (block[k] & 0xff) == 0xff) {
                printf(".");
            } else if ((block[k] & 0xff) < 32 || (block[k] & 0xff) >= 127) {
                printf("?");
            } else {
                printf("%c", block[k] & 0xff);
            }
        }
        printf("\r\n");
    }
    return 0;
}

esp_err_t i2cInitialize( void ){
    esp_err_t err;
    // I2C Master Init
    err = i2c_driver_install(i2c_port, I2C_MODE_MASTER, 
                        I2C_MASTER_RX_BUF_DISABLE, 
                        I2C_MASTER_TX_BUF_DISABLE, 0);

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency
    };
    i2c_param_config(i2c_port, &conf);
    
    
    return err;
}

void i2cDisable( void ){

        i2c_driver_delete(i2c_port);

}
