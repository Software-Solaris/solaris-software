#ifndef MPU9250_H
#define MPU9250_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

// Macros
#define SPI_HOST_USED SPI2_HOST 
#define PIN_NUM_CS         21 
#define PIN_NUM_CIPO       47
#define PIN_NUM_COPI       38
#define PIN_NUM_CLK        48

#define REG_PWR_MGMT_1     0x6B
#define REG_WHO_AM_I       0x75
#define BIT_H_RESET        0x80

typedef struct {
    spi_device_handle_t handle;
    spi_bus_config_t buscfg;
    spi_device_interface_config_t devcfg;
    spi_transaction_t trans_desc;
    uint8_t who_am_i;
    uint8_t reg;
    uint8_t data;
} mpu9250_t;

esp_err_t mpu9250_init(mpu9250_t *p_dev);
esp_err_t mpu9250_send_message(mpu9250_t *p_dev); 

#endif 
