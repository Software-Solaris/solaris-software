#ifndef ICM20948_H
#define ICM20948_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "general.h"

#define PIN_NUM_CS         21 

#define READ_OP            0x80
#define WRITE_OP           0x00

// REGISTROS
#define REG_WHO_AM_I       0x00
#define REG_USER_CTRL      0x03
#define REG_LP_CONFIG      0x05
#define REG_PWR_MGMT_1     0x06
#define REG_I2C_CTRL       0x01 // En banco 3
#define REG_SLV0_ADDR      0x03 // En banco 3
#define REG_SLV0_REG       0x04 // En banco 3
#define REG_SLV0_CTRL      0x05 // En banco 3
#define REG_ACCEL_CONFIG   0X14 // En banco 2
#define REG_GYRO_CONFIG    0x01 // En banco 2

#define REG_BANK_SEL       0x7F

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

// Registros del magnetómetro -> NOTA: Los registros del magnetómetro llegan en little endian
#define REG_MAGNETO_X_H   0x3C
#define REG_MAGNETO_X_L   0x3B


// Mensajes a enviar
#define BIT_H_RESET        0x80
#define USER_CTRL_CONFIG   0x20
#define I2C_DM_DEAC        0x00
#define I2C_SP_CONFIG      0x07
#define MAGNETO_PHYS_ADDR  0x8C // bit7 = 1 + physical address (0x0C) = 0x8C
#define MAGNETO_START_RD   0x11 // Dirección de los registros del magnetómetro
#define MAGNETO_CONFIG     0x86
#define ACCEL_FILTER_SELEC 0x31 // Pone el rango al mínimo y el filtro al máximo (modificable)
#define GYRO_FILTER_SELEC  0x31 // modificable igualmente
#define EMPTY_MESSAGE      0x00



esp_err_t icm20948_init(data_t *p_dev);
esp_err_t icm20948_config(data_t *p_dev);
esp_err_t icm20948_prepare_read(data_t *p_dev);
esp_err_t icm20948_read_measurements(data_t *p_dev);

esp_err_t icm20948_send_message(data_t *p_dev, uint8_t tx[2], uint8_t rx[2]);

#endif