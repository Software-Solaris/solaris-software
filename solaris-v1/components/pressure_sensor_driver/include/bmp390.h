#ifndef BMP390_H
#define BMP390_H

#include <stdint.h>
#include "driver/spi_master.h"  // API del ESP-IDF para SPI

//---------------------Registros y valores esperados------------------------------

#define BMP390_CHIP_ID_REG    0x00
#define BMP390_CHIP_ID_VALUE  0x60

#define BMP390_SOFT_RESET_REG 0x7E
#define BMP390_SOFT_RESET_CMD 0xB6

#define BMP390_IF_CONF_REG    0x1A
#define BMP390_IF_CONF_SPI    0x00

//------------------- Prototipos de funciones inicialización--------------------

esp_err_t bmp390_init(spi_device_handle_t *handle);

esp_err_t bmp390_write_reg(spi_device_handle_t handle, uint8_t reg, uint8_t value);
esp_err_t bmp390_read_reg(spi_device_handle_t handle, uint8_t reg, uint8_t *val);

esp_err_t bmp390_soft_reset(spi_device_handle_t handle);
esp_err_t bmp390_enable_spi_mode(spi_device_handle_t handle);

esp_err_t bmp390_read_if_conf(spi_device_handle_t handle, uint8_t *if_conf);
esp_err_t bmp390_read_chip_id(spi_device_handle_t handle, uint8_t *chip_id);

//-----------------------Activar Lecturas-----------------------

//Modo
#define BMP390_REG_PWR_CTRL     0x1B

#define BMP390_MODE_SLEEP       0x00  // 00b << 4
#define BMP390_MODE_FORCED      0x10  // 01b << 4
#define BMP390_MODE_NORMAL      0x30  // 11b << 4
#define BMP390_EN_TEMP_PRESS    0x03  // press_en + temp_en

#define BMP390_PWRCTRL_FORCED   (BMP390_MODE_FORCED  | BMP390_EN_TEMP_PRESS)
#define BMP390_PWRCTRL_NORMAL   (BMP390_MODE_NORMAL  | BMP390_EN_TEMP_PRESS)

esp_err_t bmp390_set_mode_normal(spi_device_handle_t handle);

//Oversampling
#define BMP390_REG_OSR           0x1C

#define BMP390_OSR_TEMP_x2       (1 << 3)
#define BMP390_OSR_TEMP_x8       (3 << 3)   // 011 → oversampling ×8
#define BMP390_OSR_TEMP_x16      (4 << 3)   // 100 → oversampling ×16
#define BMP390_OSR_TEMP_x32      (5 << 3)   // 101 → oversampling ×32

esp_err_t bmp390_set_osr_temp(spi_device_handle_t handle);

//Output Data Rate
#define BMP390_REG_ODR         0x1D

#define BMP390_ODR_SEL_50HZ    0x02

esp_err_t bmp390_set_odr_50hz(spi_device_handle_t handle);

//Status
#define BMP390_REG_STATUS       0x03

#define BMP390_STATUS_DRDY_TEMP  0x40
#define BMP390_STATUS_DRDY_PRES  0x20

//Filtro
#define BMP390_REG_CONFIG        0x1F

#define BMP390_IIR_COEFF_3       (2 << 1)   // 010 → IIR coeficiente 3 (más ruido pero menos tiempo)
#define BMP390_IIR_COEFF_7       (3 << 1)   // 011 → IIR coeficiente 7 (buena relación ruido-tiempo entre medidas)
#define BMP390_IIR_COEFF_15      (4 << 1)   // 100 → IIR coeficiente 15 (buena relación ruido-tiempo entre medidas)
#define BMP390_IIR_COEFF_31      (5 << 1)   // 101 → IIR coeficiente 31 (poco ruido pero mucho tiempo)

esp_err_t bmp390_set_iir(spi_device_handle_t handle);


//------------------------Lectura Temperatura--------------------------

// Dirección inicial de los coeficientes de temperatura
#define BMP390_TEMP_CALIB_REG_START  0x31  // NVM_PAR_T1<7:0>

esp_err_t bmp390_read_bytes(spi_device_handle_t handle, uint8_t start_reg, uint8_t *buffer,size_t length);
typedef struct {
    uint16_t par_t1;   // 0x31 (LSB), 0x32 (MSB)  → u16
    int16_t  par_t2;   // 0x33 (LSB), 0x34 (MSB)  → s16
    int8_t   par_t3;   // 0x35                  → s8

    float    t_lin;
} bmp390_temp_calib_t;

esp_err_t bmp390_read_temp_calibration(spi_device_handle_t handle, bmp390_temp_calib_t *tcalib);

typedef struct {
    float PAR_T1;   // Ej. ≈ 28159 / 2^8  ≈ 109.996
    float PAR_T2;   // Ej. ≈ 19960 / 2^30 ≈ 1.859e-5
    float PAR_T3;   // Ej. ≈ −7    / 2^48 ≈ −2.487e-14
} bmp390_temp_params_t;

esp_err_t bmp390_temp_params(spi_device_handle_t handle, bmp390_temp_params_t *out);

esp_err_t bmp390_read_raw_temp(spi_device_handle_t handle, uint32_t *raw_temp);

float bmp390_compensate_temperature(uint32_t raw_temp, bmp390_temp_params_t *params);

#endif  // BMP390_H
