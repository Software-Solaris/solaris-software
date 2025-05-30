#ifndef BMP390_H
#define BMP390_H

#include <stdint.h>
#include "driver/spi_master.h"  // API del ESP-IDF para SPI

// Registros y valores esperados
#define BMP390_CHIP_ID_REG    0x00
#define BMP390_CHIP_ID_VALUE  0x60

// Registro y comando para soft reset (ver datasheet BMP390)
#define BMP390_SOFT_RESET_REG 0x7E
#define BMP390_SOFT_RESET_CMD 0xB6

// Registro DATA5 (ejemplo) y valor esperado (0x80)
#define BMP390_DATA5_REG      0x09
#define BMP390_DATA5_EXPECTED 0x80

// Registro IF_CONF para configuración de la interfaz
#define BMP390_IF_CONF_REG    0x1A
// Para activar SPI (bit "spi3" = 1 y los demás en 0)
#define BMP390_IF_CONF_SPI    0x00

// Prototipos de funciones
esp_err_t bmp390_init(spi_device_handle_t *handle);
esp_err_t bmp390_soft_reset(spi_device_handle_t handle);
esp_err_t bmp390_enable_spi_mode(spi_device_handle_t handle);
esp_err_t bmp390_read_if_conf(spi_device_handle_t handle, uint8_t *if_conf);
esp_err_t bmp390_read_chip_id(spi_device_handle_t handle, uint8_t *chip_id);
esp_err_t bmp390_read_data5(spi_device_handle_t handle, uint8_t *data);

// —— Fin de tu bloque original ——

// Registros nuevos para temperatura y control
#define BMP390_REG_STATUS     0x03
#define BMP390_REG_DATA_3     0x07
#define BMP390_REG_PWR_CTRL   0x1B
#define BMP390_REG_OSR        0x1C

// Exponemos las funciones de bajo nivel para main.c
esp_err_t bmp390_write_reg(spi_device_handle_t handle, uint8_t reg_addr, uint8_t data);
esp_err_t bmp390_read_reg (spi_device_handle_t handle, uint8_t reg_addr, uint8_t *data);

// Prototipos para temperatura dinámica
esp_err_t bmp390_read_calibration(spi_device_handle_t handle);
esp_err_t bmp390_read_raw_temperature(spi_device_handle_t handle, uint32_t *uncomp_temp);
float     BMP390_compensate_temperature(uint32_t uncomp_temp);

#endif  // BMP390_H
