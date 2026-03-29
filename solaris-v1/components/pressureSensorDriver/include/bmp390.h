/**
 * @file bmp390.h
 * @brief BMP390 barometric pressure sensor driver.
 *
 * Provides SPI-based communication with the Bosch BMP390 sensor for reading
 * calibrated temperature, pressure and derived altitude. Supports data-ready
 * interrupt via GPIO event groups.
 *
 * The driver follows a three-step workflow:
 * 1. Initialise and configure the sensor (BmpInit, bmp390_aux_config,
 *    bmp390_prepare_measure).
 * 2. Wait for the data-ready interrupt (bmp390_wait_drdy) or poll status.
 * 3. Read compensated altitude/pressure/temperature (bmp390_get_altitude).
 */

#ifndef BMP390_H
#define BMP390_H

#include <stdint.h>
#include "returntypes.h"
#include "general.h"
#include "osal/eventgroups.h"
#include "osal/task.h"
#include "hal/gpio/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Initialisation Constants
 * ========================================================================= */

/**
 * @brief FreeRTOS task priority for the BMP390 initialisation task.
 */
#define K_BMP_INIT_PRIO   4

/**
 * @brief Stack size in bytes for the BMP390 initialisation task.
 */
#define K_BMP_INIT_TASK_STACK_SIZE 4096

/**
 * @brief Event group bit indicating BMP390 data-ready.
 */
#define K_BMP390_EVT_DRDY   (1u << 0)

/* ============================================================================
 * Data Types
 * ========================================================================= */

/**
 * @brief BMP390 device context.
 *
 * Groups the SPI handler, event group, ISR context and GPIO interrupt
 * configuration required to operate one BMP390 sensor instance.
 */
typedef struct {
    void* p_handler_spi;              /**< SPI device handle.                 */
    void* p_event_group;              /**< Event group for DRDY signalling.   */
    spp_gpio_isr_ctx_t isr_ctx;       /**< ISR context (event group + bits).  */
    spp_uint32_t int_pin;             /**< GPIO pin number for the interrupt. */
    spp_uint32_t int_intr_type;       /**< Interrupt trigger type.            */
    spp_uint32_t int_pull;            /**< Pull resistor: 0=none, 1=up, 2=down. */
} bmp_data_t;

/**
 * @brief Chip-select GPIO pin for the BMP390.
 */
#define K_BMP390_PIN_NUM_CS   18

/* ============================================================================
 * Initialisation
 * ========================================================================= */

/**
 * @brief Initialises the BMP390 driver.
 *
 * Creates the event group, configures the GPIO interrupt and registers the
 * ISR for data-ready signalling.
 *
 * @param[in,out] p_data Pointer to a bmp_data_t context to populate.
 */
void BmpInit(void* p_data);

/* ============================================================================
 * Configuration and Identity Check
 * ========================================================================= */

/**
 * @brief BMP390 chip ID register address.
 */
#define K_BMP390_CHIP_ID_REG    0x00

/**
 * @brief Expected value of the BMP390 chip ID register.
 */
#define K_BMP390_CHIP_ID_VALUE  0x60

/**
 * @brief BMP390 soft-reset command register address.
 */
#define K_BMP390_SOFT_RESET_REG 0x7E

/**
 * @brief Command byte that triggers a BMP390 soft reset.
 */
#define BMP390_SOFT_RESET_CMD 0xB6

/**
 * @brief BMP390 interface configuration register address.
 */
#define K_BMP390_IF_CONF_REG    0x1A

/**
 * @brief Value to select SPI 4-wire mode in IF_CONF.
 */
#define BMP390_IF_CONF_SPI    0x00

/**
 * @brief Performs a soft reset of the BMP390.
 *
 * @param[in] p_spi SPI device handle.
 * @return SPP_OK on success, error code otherwise.
 */
retval_t bmp390_soft_reset(void *p_spi);

/**
 * @brief Enables SPI 4-wire mode on the BMP390.
 *
 * @param[in] p_spi SPI device handle.
 * @return SPP_OK on success, error code otherwise.
 */
retval_t bmp390_enable_spi_mode(void *p_spi);

/**
 * @brief Reads and verifies the BMP390 chip ID.
 *
 * @param[in] p_spi SPI device handle.
 * @return SPP_OK if chip ID matches 0x60, SPP_ERROR otherwise.
 */
retval_t bmp390_config_check(void *p_spi);

/* ============================================================================
 * Measurement Configuration
 * ========================================================================= */

/** @brief Power control register address. */
#define K_BMP390_REG_PWRCTRL     0x1B
/** @brief Power control value: normal mode, pressure + temperature enabled. */
#define BMP390_VALUE_PWRCTRL   0x33

/** @brief Oversampling settings register address. */
#define K_BMP390_REG_OSR           0x1C
/** @brief Oversampling value (no oversampling, +/- 0.2 m accuracy). */
#define BMP390_VALUE_OSR         0x00

/** @brief Output data rate register address. */
#define K_BMP390_REG_ODR         0x1D
/** @brief ODR value for 50 Hz output. */
#define BMP390_VALUE_ODR       0x02

/** @brief IIR filter coefficient register address. */
#define K_BMP390_REG_IIR      0x1F
/** @brief IIR filter coefficient 1. */
#define BMP390_VALUE_IIR    0x02

/**
 * @brief Configures measurement registers (OSR, ODR, IIR, power control).
 *
 * @param[in] p_spi SPI device handle.
 * @return SPP_OK on success, error code otherwise.
 */
retval_t bmp390_prepare_measure(void* p_spi);

/** @brief Status register address. */
#define K_BMP390_REG_STATUS         0x03
/** @brief Status bit: temperature data ready. */
#define K_BMP390_STATUS_DRDY_TEMP   0x40
/** @brief Status bit: pressure data ready. */
#define BMP390_STATUS_DRDY_PRES  0x20

/**
 * @brief Blocks until the BMP390 data-ready interrupt fires.
 *
 * @param[in] p_bmp  Pointer to BMP390 device context.
 * @param[in] timeout_ms Maximum wait time in milliseconds (0 = forever).
 * @return SPP_OK if data-ready was signalled, error code on timeout.
 */
retval_t bmp390_wait_drdy(bmp_data_t* p_bmp, spp_uint32_t timeout_ms);

/* ============================================================================
 * Temperature Calibration and Reading
 * ========================================================================= */

/** @brief Start address of temperature calibration registers. */
#define K_BMP390_TEMP_CALIB_REG_START  0x31

/**
 * @brief Raw temperature calibration coefficients (as read from the sensor).
 */
typedef struct {
    uint16_t par_t1;   /**< Calibration coefficient T1 (unsigned 16-bit). */
    int16_t  par_t2;   /**< Calibration coefficient T2 (signed 16-bit).   */
    int8_t   par_t3;   /**< Calibration coefficient T3 (signed 8-bit).    */
    float    t_lin;    /**< Linearised temperature used for pressure comp. */
} bmp390_temp_calib_t;

/**
 * @brief Reads raw temperature calibration coefficients via SPI.
 *
 * @param[in]  p_spi  SPI device handle.
 * @param[out] tcalib Structure to receive the raw coefficients.
 * @return SPP_OK on success, error code otherwise.
 */
retval_t bmp390_read_raw_temp_coeffs(void *p_spi, bmp390_temp_calib_t *tcalib);

/**
 * @brief Scaled temperature calibration parameters used for compensation.
 */
typedef struct {
    float PAR_T1;   /**< Scaled T1 = raw_t1 * 2^8.   */
    float PAR_T2;   /**< Scaled T2 = raw_t2 / 2^30.   */
    float PAR_T3;   /**< Scaled T3 = raw_t3 / 2^48.   */
} bmp390_temp_params_t;

/**
 * @brief Reads and scales temperature calibration parameters.
 *
 * @param[in]  p_spi SPI device handle.
 * @param[out] out   Structure to receive scaled parameters.
 * @return SPP_OK on success, error code otherwise.
 */
retval_t bmp390_calibrate_temp_params(void *p_spi, bmp390_temp_params_t *out);

/** @brief Start address of raw temperature data registers (24-bit). */
#define K_BMP390_TEMP_RAW_REG    0x07

/**
 * @brief Reads the 24-bit raw temperature value from the sensor.
 *
 * @param[in]  p_spi    SPI device handle.
 * @param[out] raw_temp Receives the 24-bit raw ADC value.
 * @return SPP_OK on success, error code otherwise.
 */
retval_t bmp390_read_raw_temp(void *p_spi, uint32_t *raw_temp);

/**
 * @brief Compensates a raw temperature reading.
 *
 * Applies the BMP390 datasheet compensation formula using the scaled
 * calibration parameters.
 *
 * @param[in] raw_temp Raw 24-bit ADC value.
 * @param[in] params   Scaled calibration parameters.
 * @return Compensated temperature in degrees Celsius.
 */
float bmp390_compensate_temperature(spp_uint32_t raw_temp, bmp390_temp_params_t *params);

/* ============================================================================
 * Pressure Calibration and Reading
 * ========================================================================= */

/** @brief Start address of pressure calibration registers. */
#define K_BMP390_PRESS_CALIB_REG_START  0x36

/**
 * @brief Raw pressure calibration coefficients (as read from the sensor).
 */
typedef struct {
    spp_uint16_t par_p1;    /**< Coefficient P1  (unsigned 16-bit). */
    spp_uint16_t par_p2;    /**< Coefficient P2  (unsigned 16-bit). */
    spp_int8_t   par_p3;    /**< Coefficient P3  (signed 8-bit).    */
    spp_int8_t   par_p4;    /**< Coefficient P4  (signed 8-bit).    */
    spp_uint16_t par_p5;    /**< Coefficient P5  (unsigned 16-bit). */
    spp_uint16_t par_p6;    /**< Coefficient P6  (unsigned 16-bit). */
    spp_int8_t   par_p7;    /**< Coefficient P7  (signed 8-bit).    */
    spp_int8_t   par_p8;    /**< Coefficient P8  (signed 8-bit).    */
    spp_int16_t  par_p9;    /**< Coefficient P9  (signed 16-bit).   */
    spp_int8_t   par_p10;   /**< Coefficient P10 (signed 8-bit).    */
    spp_int8_t   par_p11;   /**< Coefficient P11 (signed 8-bit).    */
} bmp390_press_calib_t;

/**
 * @brief Reads raw pressure calibration coefficients via SPI.
 *
 * @param[in]  p_spi  SPI device handle.
 * @param[out] pcalib Structure to receive the raw coefficients.
 * @return SPP_OK on success, error code otherwise.
 */
retval_t bmp390_read_raw_press_coeffs(void *p_spi, bmp390_press_calib_t *pcalib);

/**
 * @brief Scaled pressure calibration parameters used for compensation.
 *
 * Each field is computed from the corresponding raw coefficient using the
 * scaling factors documented in the BMP390 datasheet.
 */
typedef struct {
    float PAR_P1;    /**< Scaled P1  = (raw - 2^14) / 2^20. */
    float PAR_P2;    /**< Scaled P2  = (raw - 2^14) / 2^29. */
    float PAR_P3;    /**< Scaled P3  = raw / 2^32.          */
    float PAR_P4;    /**< Scaled P4  = raw / 2^37.          */
    float PAR_P5;    /**< Scaled P5  = raw * 2^3.           */
    float PAR_P6;    /**< Scaled P6  = raw / 2^6.           */
    float PAR_P7;    /**< Scaled P7  = raw / 2^8.           */
    float PAR_P8;    /**< Scaled P8  = raw / 2^15.          */
    float PAR_P9;    /**< Scaled P9  = raw / 2^48.          */
    float PAR_P10;   /**< Scaled P10 = raw / 2^48.          */
    float PAR_P11;   /**< Scaled P11 = raw / 2^65.          */
} bmp390_press_params_t;

/**
 * @brief Reads and scales pressure calibration parameters.
 *
 * @param[in]  p_spi SPI device handle.
 * @param[out] out   Structure to receive scaled parameters.
 * @return SPP_OK on success, error code otherwise.
 */
retval_t bmp390_calibrate_press_params(void *p_spi, bmp390_press_params_t *out);

/** @brief Start address of raw pressure data registers (24-bit). */
#define K_BMP390_PRESS_RAW_REG    0x04

/**
 * @brief Reads the 24-bit raw pressure value from the sensor.
 *
 * @param[in]  p_spi     SPI device handle.
 * @param[out] raw_press Receives the 24-bit raw ADC value.
 * @return SPP_OK on success, error code otherwise.
 */
retval_t bmp390_read_raw_press(void *p_spi, spp_uint32_t *raw_press);

/**
 * @brief Compensates a raw pressure reading.
 *
 * Applies the BMP390 datasheet 11-coefficient polynomial compensation
 * using the linearised temperature value.
 *
 * @param[in] raw_press Raw 24-bit ADC value.
 * @param[in] t_lin     Linearised temperature from bmp390_compensate_temperature().
 * @param[in] p         Scaled calibration parameters.
 * @return Compensated pressure in Pascal.
 */
float bmp390_compensate_pressure(spp_uint32_t raw_press, float t_lin, bmp390_press_params_t *p);

/* ============================================================================
 * Altitude Calculation
 * ========================================================================= */

/**
 * @brief Reads sensors and computes barometric altitude.
 *
 * Performs the full read-compensate-calculate pipeline in one call:
 * calibrates on first invocation, reads temperature and pressure, applies
 * compensation, and derives altitude using the barometric formula
 * @f$ h = 44330 \cdot (1 - (P / 101325)^{1/5.255}) @f$.
 *
 * @param[in]  p_spi          SPI device handle.
 * @param[in]  p_bmp          BMP390 device context (reserved, may be NULL).
 * @param[out] altitude_m     Altitude in metres above sea level.
 * @param[out] pressure_pa    Compensated pressure in Pascal.
 * @param[out] temperature_c  Compensated temperature in degrees Celsius.
 * @return SPP_OK on success, SPP_ERROR_NULL_POINTER if any output pointer
 *         is NULL, or an SPI error code.
 */
retval_t bmp390_get_altitude(void *p_spi, bmp_data_t *p_bmp, float *altitude_m, float *pressure_pa, float *temperature_c);

/* ============================================================================
 * Auxiliary Helpers
 * ========================================================================= */

/**
 * @brief Performs soft reset, SPI mode enable and chip ID check.
 *
 * Convenience function that chains bmp390_soft_reset(),
 * bmp390_enable_spi_mode() and bmp390_config_check().
 *
 * @param[in] p_spi SPI device handle.
 * @return SPP_OK on success, first failing error code otherwise.
 */
retval_t bmp390_aux_config(void *p_spi);

/**
 * @brief Reads and compensates temperature in one call.
 *
 * @param[in]  p_spi       SPI device handle.
 * @param[in]  temp_params Scaled temperature calibration parameters.
 * @param[out] raw_temp    Raw 24-bit ADC value.
 * @param[out] comp_temp   Compensated temperature in degrees Celsius.
 * @return SPP_OK on success, error code otherwise.
 */
retval_t bmp390_aux_get_temp(void *p_spi, const bmp390_temp_params_t *temp_params, spp_uint32_t *raw_temp, float *comp_temp);

/**
 * @brief Reads and compensates pressure in one call.
 *
 * @param[in]  p_spi        SPI device handle.
 * @param[in]  press_params Scaled pressure calibration parameters.
 * @param[in]  t_lin        Linearised temperature for compensation.
 * @param[out] raw_press    Raw 24-bit ADC value.
 * @param[out] comp_press   Compensated pressure in Pascal.
 * @return SPP_OK on success, error code otherwise.
 */
retval_t bmp390_aux_get_press(void *p_spi, const bmp390_press_params_t *press_params, float t_lin, spp_uint32_t *raw_press, float *comp_press);

/* ============================================================================
 * Interrupt Configuration
 * ========================================================================= */

/** @brief Interrupt control register address. */
#define K_BMP390_REG_INT_CTRL     0x19
/** @brief INT_CTRL bit: enable data-ready interrupt output. */
#define K_BMP390_INT_CTRL_DRDY_EN 0x40
/** @brief INT_CTRL bit: active-high interrupt level. */
#define K_BMP390_INT_CTRL_LEVEL   0x02

/**
 * @brief Enables the data-ready interrupt output on the BMP390 INT pin.
 *
 * Configures active-high, non-latched DRDY interrupt.
 *
 * @param[in] p_spi SPI device handle.
 * @return SPP_OK on success, error code otherwise.
 */
retval_t bmp390_int_enable_drdy(void *p_spi);

#ifdef __cplusplus
}
#endif

#endif /* BMP390_H */
