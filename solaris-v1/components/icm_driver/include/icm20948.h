#ifndef ICM20948_H
#define ICM20948_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "macros.h"

#define PIN_NUM_CS         21 

#define READ_OP            0x80
#define WRITE_OP           0x00

#define REG_PWR_MGMT_1     0x06
#define REG_WHO_AM_I       0x00

#define START_CONECTION    0x00 //Para iniciar la conexión, se pone el CS a 0x00
#define BIT_H_RESET        0x80
#define EMPTY_MESSAGE      0x00


esp_err_t icm20948_init(data_t *p_dev);
esp_err_t icm20948_send_message(data_t *p_dev, uint8_t tx[2], uint8_t rx[2]);
esp_err_t icm20948_get_measurements(data_t *p_dev);

#endif