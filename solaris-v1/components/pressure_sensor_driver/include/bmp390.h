#ifndef BMP390_H
#define BMP390_H

#include <stdint.h>
#include "returntypes.h"
#include "general.h"
#include "osal/eventgroups.h"
#include "osal/task.h"
#include "hal/gpio/gpio.h" 

///--------------------Read/Write Operators------------------------------
#define READ_OP            0x80
#define EMPTY_MESSAGE      0x00

//---------------------INIT------------------------------
#define BMP_INIT_PRIO   4
#define BMP_INIT_TASK_STACK_SIZE 1024
#define BMP390_EVT_DRDY   (1u << 0) 

typedef struct {
    void* p_handler_spi;    

    void* p_event_group;    

    spp_gpio_isr_ctx_t isr_ctx; 

    spp_uint32_t int_pin;       
    spp_uint32_t int_intr_type;  
    spp_uint32_t int_pull;       // 0 none, 1 pullup, 2 pulldown
} bmp_data_t;

#define PIN_NUM_CS   18

void BmpInit(void* p_data);

//--------------------CONFIG and CHECK---------------------------
#define BMP390_CHIP_ID_REG    0x00
#define BMP390_CHIP_ID_VALUE  0x60

#define BMP390_SOFT_RESET_REG 0x7E
#define BMP390_SOFT_RESET_CMD 0xB6

#define BMP390_IF_CONF_REG    0x1A
#define BMP390_IF_CONF_SPI    0x00

retval_t bmp390_soft_reset(void *p_spi);
retval_t bmp390_enable_spi_mode(void *p_spi);

retval_t bmp390_config_check(void *p_spi);

//-----------------------PREPARE READ-----------------------
//Modo
#define BMP390_REG_PWRCTRL     0x1B
#define BMP390_VALUE_PWRCTRL   0x33   // (0x30 | 0x01 | 0x02) = 0x33 (normal|press_en|temp_en)

//Oversampling
#define BMP390_REG_OSR           0x1C
#define BMP390_VALUE_OSR         0x00 // Adjust according to what we need (precision-time-energy) with this +-0.2m

//Output Data Rate
#define BMP390_REG_ODR         0x1D
#define BMP390_VALUE_ODR       0x02 // 50Hz

//Filtro
#define BMP390_REG_IIR      0x1F
#define BMP390_VALUE_IIR    0x02 // Coefficient 1

retval_t bmp390_prepare_measure(void* p_spi);

//Status
#define BMP390_REG_STATUS         0x03

#define BMP390_STATUS_DRDY_TEMP   0x40
#define BMP390_STATUS_DRDY_PRES  0x20

retval_t bmp390_wait_drdy(bmp_data_t* p_bmp, spp_uint32_t timeout_ms);

//------------------------READ TEMP--------------------------
#define BMP390_TEMP_CALIB_REG_START  0x31
typedef struct {
    uint16_t par_t1;   // 0x31 (LSB), 0x32 (MSB)  -> u16
    int16_t  par_t2;   // 0x33 (LSB), 0x34 (MSB)  -> s16
    int8_t   par_t3;   // 0x35                    -> s8

    float    t_lin;
} bmp390_temp_calib_t;

retval_t bmp390_read_raw_temp_coeffs(void *p_spi, bmp390_temp_calib_t *tcalib);

typedef struct {
    float PAR_T1;   
    float PAR_T2;   
    float PAR_T3; 
} bmp390_temp_params_t;

retval_t bmp390_calibrate_temp_params(void *p_spi, bmp390_temp_params_t *out);

#define BMP390_TEMP_RAW_REG    0x07
retval_t bmp390_read_raw_temp(void *p_spi, uint32_t *raw_temp);

float bmp390_compensate_temperature(spp_uint32_t raw_temp, bmp390_temp_params_t *params);

//------------------------READ PRESS--------------------------
#define BMP390_PRESS_CALIB_REG_START  0x36
typedef struct {
    spp_uint16_t par_p1;   // 0x36 (LSB), 0x37 (MSB)
    spp_uint16_t par_p2;   // 0x38 (LSB), 0x39 (MSB)
    spp_int8_t   par_p3;   // 0x3A
    spp_int8_t   par_p4;   // 0x3B
    spp_uint16_t  par_p5;   // 0x3C (LSB), 0x3D (MSB)
    spp_uint16_t   par_p6;   // 0x3E
    spp_int8_t   par_p7;   // 0x40
    spp_int8_t   par_p8;   // 0x41
    spp_int16_t  par_p9;   // 0x42 (LSB), 0x43 (MSB)
    spp_int8_t   par_p10;  // 0x44
    spp_int8_t   par_p11;  // 0x45
} bmp390_press_calib_t;

retval_t bmp390_read_raw_press_coeffs(void *p_spi, bmp390_press_calib_t *pcalib);

typedef struct {
    float PAR_P1;   // = (raw.par_p1 - 2^14) / 2^20
    float PAR_P2;   // = (raw.par_p2 - 2^14) / 2^29
    float PAR_P3;   // = raw.par_p3 / 2^32
    float PAR_P4;   // = raw.par_p4 / 2^37
    float PAR_P5;   // = raw.par_p5 / 2^-3
    float PAR_P6;   // = raw.par_p6 / 2^6
    float PAR_P7;   // = raw.par_p7 / 2^8
    float PAR_P8;   // = raw.par_p8 / 2^15
    float PAR_P9;   // = raw.par_p9 / 2^48
    float PAR_P10;  // = raw.par_p10 / 2^48
    float PAR_P11;  // = raw.par_p11 / 2^65
} bmp390_press_params_t;

retval_t bmp390_calibrate_press_params(void *p_spi, bmp390_press_params_t *out);

#define BMP390_PRESS_RAW_REG    0x04
retval_t bmp390_read_raw_press(void *p_spi, spp_uint32_t *raw_press);

float bmp390_compensate_pressure(spp_uint32_t raw_press, float t_lin, bmp390_press_params_t *p);

//--------------------CALCULATE ALTITUDE---------------------------
retval_t bmp390_get_altitude(void *p_spi, bmp_data_t *p_bmp, float *altitude);

//-----------Aux Functions-----------
retval_t bmp390_aux_config(void *p_spi);
retval_t bmp390_aux_get_temp(void *p_spi, const bmp390_temp_params_t *temp_params, spp_uint32_t *raw_temp, float *comp_temp);
retval_t bmp390_aux_get_press(void *p_spi, const bmp390_press_params_t *press_params, float t_lin, spp_uint32_t *raw_press, float *comp_press);


#endif  // BMP390_H