#ifndef MACROS_H
#define MACROS_H

#include <driver/spi_master.h>

// Macros
#define SPI_HOST_USED SPI2_HOST 
#define PIN_NUM_CIPO       47
#define PIN_NUM_COPI       38
#define PIN_NUM_CLK        48

typedef struct {
    spi_device_handle_t handle;
    spi_bus_config_t buscfg;
    spi_device_interface_config_t devcfg;
    spi_transaction_t trans_desc;
    uint8_t sensor_id;
    uint8_t reg;
    uint8_t data;
} data_t;

#endif
