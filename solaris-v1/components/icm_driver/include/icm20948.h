#ifndef ICM20948_H
#define ICM20948_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "macros.h"

//-----------------------------PINS (Hardware)-----------------------------
#define SPI_HOST_USED SPI2_HOST 
#define PIN_NUM_CS         21 
#define PIN_NUM_CIPO       47
#define PIN_NUM_COPI       38
#define PIN_NUM_CLK        48

//-----------------------------OPERATION TYPE-----------------------------
#define READ_OP            0x80
#define WRITE_OP           0x00

//-----------------------------ICM REGISTERS-----------------------------
#define REG_WHO_AM_I       0x00
#define REG_USER_CTRL      0x03
#define REG_LP_CONFIG      0x05
#define REG_PWR_MGMT_1     0x06
#define REG_I2C_CTRL       0x01 // On data bank 3
#define REG_SLV0_ADDR      0x03 // On data bank 3
#define REG_SLV0_REG       0x04 // On data bank 3
#define REG_SLV0_CTRL      0x05 // On data bank 3
#define REG_SLV0_DO        0x06 // On data bank 3
#define REG_SLV4_ADDR      0x13 // On data bank 3
#define REG_SLV4_REG       0x14 // On data bank 3
#define REG_SLV4_CTRL      0x15 // On data bank 3
#define REG_SLV4_DO        0x16 // On data bank 3
#define REG_SLV4_DI        0x17 // On data bank 3
#define REG_ACCEL_CONFIG   0X14 // On data bank 2
#define REG_GYRO_CONFIG    0x01 // On data bank 2

#define REG_BANK_SEL       0x7F

//-----------------------------ACCELEROMETER-----------------------------
#define REG_ACCEL_X_H     0x2D
#define REG_ACCEL_X_L     0x2E
#define REG_ACCEL_Y_H     0x2F
#define REG_ACCEL_Y_L     0x30
#define REG_ACCEL_Z_H     0x31
#define REG_ACCEL_Z_L     0x32

//-----------------------------GYROSCOPE-----------------------------
#define REG_GYRO_X_H      0x33
#define REG_GYRO_X_L      0x34
#define REG_GYRO_Y_H      0x35
#define REG_GYRO_Y_L      0x36
#define REG_GYRO_Z_H      0x37
#define REG_GYRO_Z_L      0x38

//-----------------------------MAGNETOMETER-----------------------------
#define REG_MAGNETO_X_H   0x3D
#define REG_MAGNETO_X_L   0x3C
#define REG_MAGNETO_Y_H   0x3F
#define REG_MAGNETO_Y_L   0x3E
#define REG_MAGNETO_Z_H   0x41
#define REG_MAGNETO_Z_L   0x40


//-----------------------------MESSAGES-----------------------------
#define BIT_H_RESET        0x80
#define USER_CTRL_CONFIG   0x30
#define I2C_DM_DEAC        0x00
#define I2C_SP_CONFIG      0x07
#define MAGNETO_WR_ADDR    0x0C
#define MAGNETO_RD_ADDR    0x8C // bit7 = 1 + physical address (0x0C) = 0x8C
#define MAGNETO_START_RD   0x10 // Magnetometer address of data (ST1 + DATA) on magnetometer registers
#define MAGNETO_WHO_AM_I   0x01
#define MAGNETO_CTRL_2     0x31 // Control2 magnetometer address on magnetometer registers
#define MAGNETO_CONFIG_1   0x80
#define MAGNETO_CONFIG_2   0x89
#define MAGNETO_MSM_MODE_2 0x04   
#define ACCEL_FILTER_SELEC 0x31 // Puts range on minimum value and filter on maximum
#define GYRO_FILTER_SELEC  0x31 // Puts range on minimum value and filter on maximum
#define EMPTY_MESSAGE      0x00


//-----------------------------FUNCTIONS-----------------------------
esp_err_t icm20948_init(data_t *p_dev);
esp_err_t icm20948_config(data_t *p_dev);
esp_err_t icm20948_prepare_read(data_t *p_dev);
esp_err_t icm20948_read_measurements(data_t *p_dev);

esp_err_t icm20948_send_message(data_t *p_dev, uint8_t tx[2], uint8_t rx[2]);

#endif