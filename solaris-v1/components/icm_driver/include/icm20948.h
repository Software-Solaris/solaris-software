#ifndef ICM20948_H
#define ICM20948_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "macros.h"

#define PIN_NUM_CS         21 

#define READ_OP            0x80
#define WRITE_OP           0x00

// REGISTROS
#define REG_PWR_MGMT_1     0x06
#define REG_WHO_AM_I       0x00
#define REG_LP_CONFIG      0x05
#define REG_BANK_SEL_0     0x7F

// Registros del acelerómetro
#define REG_ACCEL_X_H     0x2D
#define REG_ACCEL_X_L     0x2E
#define REG_ACCEL_Y_H     0x2F
#define REG_ACCEL_Y_L     0x30
#define REG_ACCEL_Z_H     0x31
#define REG_ACCEL_Z_L     0x32

// Registros del giroscopio
#define REG_GYRO_X_H      0x33
#define REG_GYRO_X_L      0x34
#define REG_GYRO_Y_H      0x35
#define REG_GYRO_Y_L      0x36
#define REG_GYRO_Z_H      0x37
#define REG_GYRO_Z_L      0x38


// Mensajes a enviar
#define BIT_H_RESET        0x80
#define I2C_DEAC           0x00
#define EMPTY_MESSAGE      0x00



esp_err_t icm20948_init(data_t *p_dev);
esp_err_t icm20948_config(data_t *p_dev);
esp_err_t icm20948_get_measurements(data_t *p_dev);

esp_err_t icm20948_send_message(data_t *p_dev, uint8_t tx[2], uint8_t rx[2]);

#endif