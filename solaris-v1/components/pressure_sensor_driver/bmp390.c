#include "bmp390.h"
#include <string.h>
#include <math.h>
#include "spi.h"
#include "osal/task.h"
#include "spp_log.h"

static const char* TAG = "BMP390";
spp_uint8_t id, ifc;
bmp390_temp_calib_t raw_calib;
bmp390_temp_params_t temp_params;
spp_uint32_t raw_temp;
bmp390_press_calib_t raw_press_calib;
bmp390_press_params_t press_params;
spp_uint32_t raw_press;
float t_lin;
spp_uint8_t st;
float partial_data1, partial_data2, partial_data3, partial_data4;
float partial_out1, partial_out2;
float comp_press;

//--------------------INIT (8 dummy bits and halfduplex)---------------------------

/**
 * @brief Initializes the BMP390 pressure sensor driver.
 * 
 * This function performs the complete initialization of the BMP390 sensor,
 * including SPI communication setup, event group creation, and GPIO interrupt
 * configuration. It must be called before any sensor operations.
 * 
 * @param[in,out] p_data Pointer to a bmp_data_t structure that will be
 *                        populated with initialization data including:
 *                        - SPI handler
 *                        - Event group pointer
 *                        - ISR context with event group and ready bits
 * 
 * @return void
 * 
 * @details
 * The initialization sequence includes:
 * 1. Retrieves and initializes the SPI handler
 * 2. Obtains event group buffer and creates an event group
 * 3. Configures ISR context with the event group and BMP390_EVT_DRDY bit
 * 4. Sets up GPIO interrupt on the sensor's interrupt pin with specified
 *    interrupt type and pull configuration
 * 5. Registers the ISR callback for the interrupt pin
 * 6. Deletes the current task upon completion
 */
void BmpInit(void* p_data)
{
    bmp_data_t* p_bmp = (bmp_data_t*)p_data;

    void* p_buffer_eg;

    p_buffer_eg = SPP_OSAL_GetEventGroupsBuffer();
    p_bmp->p_event_group = SPP_OSAL_EventGroupCreate(p_buffer_eg);

    p_bmp->isr_ctx.event_group = p_bmp->p_event_group;
    p_bmp->isr_ctx.bits        = BMP390_EVT_DRDY;

    SPP_HAL_GPIO_ConfigInterrupt(p_bmp->int_pin, p_bmp->int_intr_type, p_bmp->int_pull);
    SPP_HAL_GPIO_RegisterISR(p_bmp->int_pin, (void*)&p_bmp->isr_ctx);
}


//--------------------CONFIG and CHECK---------------------------
/**
 * @brief Performs a soft reset of the BMP390 pressure sensor.
 * 
 * @param[in] p_spi Pointer to the SPI device handle.
 * @return retval_t Status code indicating success or failure.
 */
retval_t bmp390_soft_reset(void *p_spi)
{
    spp_uint8_t buf[2] = 
    {
        (spp_uint8_t)BMP390_SOFT_RESET_REG,
        (spp_uint8_t)BMP390_SOFT_RESET_CMD
    };

    retval_t ret = SPP_HAL_SPI_Transmit(p_spi, buf, (spp_uint8_t)sizeof(buf));
    SPP_OSAL_TaskDelay(100);

    return ret;
}

/**
 * @brief Enables SPI 4-wire mode on the BMP390 sensor.
 * 
 * @param[in] p_spi Pointer to the SPI device handle.
 * @return retval_t Status code indicating success or failure.
 */
retval_t bmp390_enable_spi_mode(void *p_spi)
{
    spp_uint8_t buf[2] = 
    {
        (spp_uint8_t)BMP390_IF_CONF_REG,
        (spp_uint8_t)BMP390_IF_CONF_SPI
    };

    retval_t ret = SPP_HAL_SPI_Transmit(p_spi, buf, (spp_uint8_t)sizeof(buf));
    SPP_OSAL_TaskDelay(100);

    return ret;
}

/**
 * @brief Verifies the configuration of the BMP390 sensor.
 * 
 * @param[in] p_spi Pointer to the SPI device handle.
 * @return retval_t Status code indicating success or failure.
 */
retval_t bmp390_config_check(void *p_spi)
{
    spp_uint8_t buf[9] = 
    {
        (spp_uint8_t)(READ_OP | BMP390_IF_CONF_REG),    EMPTY_MESSAGE, EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | BMP390_SOFT_RESET_REG), EMPTY_MESSAGE, EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | BMP390_CHIP_ID_REG),    EMPTY_MESSAGE, EMPTY_MESSAGE
    };
    
    retval_t ret;
    ret = SPP_HAL_SPI_Transmit(p_spi, buf, (spp_uint8_t)sizeof(buf));
    
    if (ret != SPP_OK) {
        return ret;
    }

    SPP_LOGI(TAG, "Buffer: 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X", buf[0], buf[2], buf[4], buf[1], buf[3], buf[5]);

    if (buf[5] != 0x60) {
        SPP_LOGE(TAG, "BMP390 not detected! Expected ID: 0x%02X, Read ID: 0x%02X", 0x60, buf[5]);
        return SPP_ERROR;
    }

    return ret;
}

/**
 * @brief Auxiliar function for configuring the BMP390 sensor.
 * 
 * @param[in] p_spi Pointer to the SPI device handle.
 * @return retval_t Status code from the initialization sequence.
 */
retval_t bmp390_aux_config(void *p_spi)
{
    retval_t ret;

    ret = bmp390_soft_reset(p_spi);
    if (ret != SPP_OK) return ret;

    ret = bmp390_enable_spi_mode(p_spi);
    if (ret != SPP_OK) return ret;

    ret = bmp390_config_check(p_spi);
    if (ret != SPP_OK) return ret;
    
    return ret;
}

//--------------------PREPARE READ---------------------------
/**
 * @brief Prepares the BMP390 sensor for measurement by configuring its registers.
 * 
 * @details Configuration registers sent:
 *   - BMP390_REG_OSR: Oversampling Settings Register
 *   - BMP390_REG_ODR: Output Data Rate Register
 *   - BMP390_REG_IIR: IIR Filter Register
 *   - BMP390_REG_PWRCTRL: Power Control Register
 * 
 * @param[in] p_spi Pointer to the SPI device handler used for communication with the sensor.
 * 
 * @return retval_t Status code indicating success or failure of the SPI transmission.
 */
retval_t bmp390_prepare_measure(void *p_spi)
{
    spp_uint8_t buf[8] = 
    {
        (spp_uint8_t)BMP390_REG_OSR,     (spp_uint8_t)BMP390_VALUE_OSR,
        (spp_uint8_t)BMP390_REG_ODR,     (spp_uint8_t)BMP390_VALUE_ODR,
        (spp_uint8_t)BMP390_REG_IIR,     (spp_uint8_t)BMP390_VALUE_IIR,
        (spp_uint8_t)BMP390_REG_PWRCTRL, (spp_uint8_t)BMP390_VALUE_PWRCTRL
    };

    retval_t ret = SPP_HAL_SPI_Transmit(p_spi, buf, sizeof(buf));

    return ret;
}

/**
 * @brief Wait for BMP390 data ready interrupt
 * 
 * @param[in] p_bmp Pointer to BMP390 device context
 * @param[in] timeout_ms Timeout in milliseconds (0 = wait indefinitely)
 * 
 * @return SPP_OK if data ready event was signaled, error code otherwise
 */
retval_t bmp390_wait_drdy(bmp_data_t* p_bmp, spp_uint32_t timeout_ms)
{
    osal_eventbits_t bits;

    retval_t ret = OSAL_EventGroupWaitBits(
        p_bmp->p_event_group,
        BMP390_EVT_DRDY,
        1,      // clear_on_exit
        0,      // wait_for_all_bits
        timeout_ms,
        &bits
    );

    return ret;
}

//--------------------READ TEMP---------------------------
/**
 * @brief Reads raw temperature calibration coefficients from BMP390 sensor.
 *
 * @param[in] p_spi Pointer to SPI device handle.
 * @param[out] tcalib Pointer to temperature calibration structure to be filled.
 *
 * @return retval_t Status code indicating success or failure of the SPI transmission.
 */
retval_t bmp390_read_raw_temp_coeffs(void *p_spi, bmp390_temp_calib_t *tcalib)
{
    retval_t ret;

    spp_uint8_t buf[10] = {
        (spp_uint8_t)(READ_OP | (BMP390_TEMP_CALIB_REG_START + 0)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_TEMP_CALIB_REG_START + 1)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_TEMP_CALIB_REG_START + 2)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_TEMP_CALIB_REG_START + 3)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_TEMP_CALIB_REG_START + 4)), EMPTY_MESSAGE
    };

    ret = SPP_HAL_SPI_Transmit(p_spi, buf, sizeof(buf));
    if (ret != SPP_OK) {
        return ret;
    }

    spp_uint8_t raw[5];
    raw[0] = buf[1];
    raw[1] = buf[3];
    raw[2] = buf[5];
    raw[3] = buf[7];
    raw[4] = buf[9];

    tcalib->par_t1 = (spp_uint16_t)((raw[1] << 8) | raw[0]);
    tcalib->par_t2 = (spp_int16_t)((raw[3] << 8) | raw[2]);
    tcalib->par_t3 = (spp_int8_t)raw[4];

    return ret;
}

/**
 * @brief Calibrate temperature parameters from BMP390 sensor coefficients
 * 
 * @param[in] p_spi Pointer to SPI interface for sensor communication
 * @param[out] out Pointer to temperature parameters structure to be populated
 * 
 * @return retval_t Status code indicating success or failure of the SPI transmission.
 */
retval_t bmp390_calibrate_temp_params(void *p_spi, bmp390_temp_params_t *out)
{
    retval_t ret;
    bmp390_temp_calib_t raw;

    ret = bmp390_read_raw_temp_coeffs(p_spi, &raw);
    if (ret != SPP_OK) {
        return ret;
    }

    out->PAR_T1 = raw.par_t1 * 256.0f;                   // 2^(-8)
    out->PAR_T2 = raw.par_t2 / 1073741824.0f;            // 2^30
    out->PAR_T3 = raw.par_t3 / 281474976710656.0f;       // 2^48

    return ret;
}

/**
 * @brief Read raw temperature data from BMP390 sensor via SPI.
 *
 * @param[in] p_spi Pointer to SPI device handle
 * @param[out] raw_temp Pointer to store the 24-bit raw temperature value
 *
 * @return retval_t Status code indicating success or failure of the SPI transmission.
 */
retval_t bmp390_read_raw_temp(void *p_spi, uint32_t *raw_temp)
{
    retval_t ret;

    spp_uint8_t buf[6] = {
        (spp_uint8_t)(READ_OP | (BMP390_TEMP_RAW_REG + 0)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_TEMP_RAW_REG + 1)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_TEMP_RAW_REG + 2)), EMPTY_MESSAGE
    };

    ret = SPP_HAL_SPI_Transmit(p_spi, buf, sizeof(buf));
    if (ret != SPP_OK) {
        return ret;
    }

    spp_uint8_t xlsb = buf[1];
    spp_uint8_t lsb  = buf[3];
    spp_uint8_t msb  = buf[5];

    *raw_temp = ((spp_uint32_t)msb << 16) | ((spp_uint32_t)lsb <<  8) | (spp_uint32_t)xlsb;

    return ret;
}

/**
 * @brief Compensates raw temperature reading from BMP390 sensor
 * 
 * @param[in] raw_temp Raw temperature value from sensor ADC
 * @param[in] params Pointer to BMP390 temperature calibration parameters
 * 
 * @return Compensated temperature value (float) in Celsius.
 */
float bmp390_compensate_temperature(spp_uint32_t raw_temp, bmp390_temp_params_t *params)
{
    float partial1 = (float)raw_temp - params->PAR_T1;
    float partial2 = partial1 * params->PAR_T2;
    float t_lin = partial2 + (partial1 * partial1) * params->PAR_T3;

    return t_lin;
}

/**
 * @brief Reads raw temperature and applies compensation to get actual temperature value.
 *
 * @param[in] p_spi Pointer to SPI interface for communication with BMP390 sensor.
 * @param[in] temp_params Pointer to temperature calibration parameters.
 * @param[out] raw_temp Pointer to store raw temperature data from sensor.
 * @param[out] comp_temp Pointer to store compensated temperature value in degrees Celsius.
 *
 * @return SPP_OK on success, error code otherwise.
 */
retval_t bmp390_aux_get_temp(void *p_spi, const bmp390_temp_params_t *temp_params, spp_uint32_t *raw_temp, float *comp_temp)
{
    retval_t ret;

    ret = bmp390_read_raw_temp(p_spi, raw_temp);
    if (ret != SPP_OK) {
        return ret;
    }

    *comp_temp = bmp390_compensate_temperature(*raw_temp, (bmp390_temp_params_t*)temp_params);

    return ret;
}

//--------------------READ PRESS---------------------------
/**
 * @brief Reads raw pressure calibration coefficients from BMP390 sensor.
 *
 * @param[in] p_spi Pointer to SPI device handle.
 * @param[out] pcalib Pointer to pressure calibration structure to be filled.
 *
 * @return retval_t Status code indicating success or failure of the SPI transmission.
 */
retval_t bmp390_read_raw_press_coeffs(void *p_spi, bmp390_press_calib_t *pcalib)
{
    retval_t ret;

    spp_uint8_t buf[32] = {
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START +  0)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START +  1)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START +  2)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START +  3)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START +  4)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START +  5)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START +  6)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START +  7)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START +  8)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START +  9)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START + 10)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START + 11)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START + 12)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START + 13)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START + 14)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_CALIB_REG_START + 15)), EMPTY_MESSAGE
    };

    ret = SPP_HAL_SPI_Transmit(p_spi, buf, sizeof(buf));
    if (ret != SPP_OK) return ret;

    spp_uint8_t raw[16];
    for (int i = 0; i < 16; i++) {
        raw[i] = buf[2 * i + 1];
    }

    pcalib->par_p1  = (spp_uint16_t)((raw[1] << 8) | raw[0]);
    pcalib->par_p2  = (spp_uint16_t)((raw[3] << 8) | raw[2]);
    pcalib->par_p3  = (spp_int8_t)   raw[4];
    pcalib->par_p4  = (spp_int8_t)   raw[5];
    pcalib->par_p5  = (spp_uint16_t)((raw[7] << 8) | raw[6]);
    pcalib->par_p6  = (spp_uint16_t)((raw[9] << 8) | raw[8]);
    pcalib->par_p7  = (spp_int8_t)   raw[10];
    pcalib->par_p8  = (spp_int8_t)   raw[11];
    pcalib->par_p9  = (spp_int16_t)((raw[13] << 8) | raw[12]);
    pcalib->par_p10 = (spp_int8_t)   raw[14];
    pcalib->par_p11 = (spp_int8_t)   raw[15];

    return ret;
}

/**
 * @brief Calibrate pressure parameters from BMP390 sensor coefficients
 * 
 * @param[in] p_spi Pointer to SPI interface for sensor communication
 * @param[out] out Pointer to pressure parameters structure to be populated
 * 
 * @return retval_t Status code indicating success or failure of the SPI transmission.
 */
retval_t bmp390_calibrate_press_params(void *p_spi, bmp390_press_params_t *out)
{
    retval_t ret;
    bmp390_press_calib_t raw;

    ret = bmp390_read_raw_press_coeffs(p_spi, &raw);
    if (ret != SPP_OK) {
        return ret;
    }

    out->PAR_P1  = (raw.par_p1  - 16384.0f) / 1048576.0f;             // (p1 - 2^14) / 2^20
    out->PAR_P2  = (raw.par_p2  - 16384.0f) / 536870912.0f;           // (p2 - 2^14) / 2^29
    out->PAR_P3  =  raw.par_p3 / 4294967296.0f;                       // / 2^32
    out->PAR_P4  =  raw.par_p4 / 137438953472.0f;                     // / 2^37
    out->PAR_P5  =  raw.par_p5 * 8.0f;                                // / 2^-3
    out->PAR_P6  =  raw.par_p6 / 64.0f;                               // / 2^6
    out->PAR_P7  =  raw.par_p7 / 256.0f;                              // / 2^8
    out->PAR_P8  =  raw.par_p8 / 32768.0f;                            // / 2^15
    out->PAR_P9  =  raw.par_p9 / 281474976710656.0f;                  // / 2^48
    out->PAR_P10 =  raw.par_p10 / 281474976710656.0f;                 // / 2^48
    out->PAR_P11 =  raw.par_p11 / 36893488147419103232.0f;            // / 2^65

    return ret;
}
/**
 * @brief Read raw pressure data from BMP390 sensor via SPI.
 *
 * @param[in] p_spi Pointer to SPI device handle
 * @param[out] raw_press Pointer to store the 24-bit raw pressure value
 *
 * @return retval_t Status code indicating success or failure of the SPI transmission.
 */
retval_t bmp390_read_raw_press(void *p_spi, spp_uint32_t *raw_press)
{
    retval_t ret;

    spp_uint8_t buf[6] = {
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_RAW_REG + 0)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_RAW_REG + 1)), EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | (BMP390_PRESS_RAW_REG + 2)), EMPTY_MESSAGE
    };

    ret = SPP_HAL_SPI_Transmit(p_spi, buf, sizeof(buf));
    if (ret != SPP_OK) {
        return ret;
    }

    spp_uint8_t xlsb = buf[1];
    spp_uint8_t lsb  = buf[3];
    spp_uint8_t msb  = buf[5];

    *raw_press = ((spp_uint32_t)msb << 16) | ((spp_uint32_t)lsb <<  8) | (spp_uint32_t)xlsb;

    return ret;
}

/**
 * @brief Compensates raw pressure reading from BMP390 sensor
 * 
 * @param[in] raw_press Raw pressure value from sensor ADC
 * @param[in] params Pointer to BMP390 pressure calibration parameters
 * 
 * @return Compensated pressure value (float) in Pascal.
 */
float bmp390_compensate_pressure(spp_uint32_t raw_press, float t_lin, bmp390_press_params_t *p)
{
    partial_data1 = p->PAR_P6 * t_lin;
    partial_data2 = p->PAR_P7 * (t_lin * t_lin);
    partial_data3 = p->PAR_P8 * (t_lin * t_lin * t_lin);
    partial_out1  = p->PAR_P5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = p->PAR_P2 * t_lin;
    partial_data2 = p->PAR_P3 * (t_lin * t_lin);
    partial_data3 = p->PAR_P4 * (t_lin * t_lin * t_lin);
    partial_out2  = raw_press * (p->PAR_P1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = raw_press * raw_press;                               
    partial_data2 = p->PAR_P9 + p->PAR_P10 * t_lin;                     
    partial_data3 = partial_data1 * partial_data2;                      
    partial_data4 = partial_data3 + (raw_press * raw_press * raw_press) * p->PAR_P11;   

    comp_press = partial_out1 + partial_out2 + partial_data4;

    return comp_press;
}


/**
 * @brief Reads and compensates raw pressure data from BMP390 sensor.
 *
 * @param[in] p_spi Pointer to SPI interface handle.
 * @param[in] press_params Pointer to pressure sensor parameters structure.
 * @param[in] t_lin Linearized temperature value for compensation.
 * @param[out] raw_press Pointer to store raw pressure reading.
 * @param[out] comp_press Pointer to store compensated pressure value.
 *
 * @return retval_t Status code indicating success or failure of the SPI transmission.
 */
retval_t bmp390_aux_get_press(void *p_spi, const bmp390_press_params_t *press_params, float t_lin, spp_uint32_t *raw_press, float *comp_press)
{
    retval_t ret;

    ret = bmp390_read_raw_press(p_spi, raw_press);
    if (ret != SPP_OK) {
        return ret;
    }

    *comp_press = bmp390_compensate_pressure(*raw_press, t_lin, (bmp390_press_params_t*)press_params);

    return ret;
}

//--------------------CALCULATE ALTITUDE---------------------------

/**
 * @brief Calculates altitude from barometric pressure readings
 * 
 * Retrieves calibrated temperature and pressure data from the BMP390 sensor,
 * computes compensated pressure, and derives altitude using the barometric
 * altitude formula.
 * 
 * @param[in] p_spi Pointer to SPI interface handle
 * @param[in] p_bmp Pointer to BMP390 device structure
 * @param[out] altitude Pointer to store calculated altitude in meters
 * 
 * @return retval_t Status code indicating success or failure of the SPI transmission.
 */
retval_t bmp390_get_altitude(void *p_spi, bmp_data_t *p_bmp, float *altitude)
{
    retval_t ret;
    float t_lin;
    float comp_press;

    static spp_bool_t s_inited = false;
    static bmp390_temp_params_t temp_params_static;
    static bmp390_press_params_t press_params_static;
    static spp_uint32_t raw_temp_static;
    static spp_uint32_t raw_press_static;

    if (s_inited == false)
    {
        ret = bmp390_calibrate_temp_params(p_spi, &temp_params_static);
        if (ret != SPP_OK) return ret;

        ret = bmp390_calibrate_press_params(p_spi, &press_params_static);
        if (ret != SPP_OK) return ret;

        s_inited = true;
    }

    ret = bmp390_wait_drdy(p_bmp, 1000);
    if (ret != SPP_OK) {
        return ret;
    }

    ret = bmp390_aux_get_temp(p_spi, &temp_params_static, &raw_temp_static, &t_lin);
    if (ret != SPP_OK) {
        return ret;
    }

    ret = bmp390_aux_get_press(p_spi, &press_params_static, t_lin, &raw_press_static, &comp_press);
    if (ret != SPP_OK) {
        return ret;
    }

    *altitude = 44330.0f * (1.0f - powf(comp_press / 101325.0f, 1.0f / 5.255f));

    return ret;
}
