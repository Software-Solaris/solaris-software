#include "bmp390.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

#include "core/returntypes.h"
#include "spi.h"
#include "task.h"

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

void BmpInit(void* p_data)
{
    bmp_data_t* p_bmp = (bmp_data_t*)p_data;

    void* p_spi_bmp;
    void* p_buffer_eg;

    /* --- SPI bus + device --- */
    SPP_HAL_SPI_BusInit();

    p_spi_bmp = SPP_HAL_SPI_GetHandler();
    SPP_HAL_SPI_DeviceInit(p_spi_bmp);

    p_bmp->p_handler_spi = p_spi_bmp;

    /* --- EventGroup + ISR context --- */
    p_buffer_eg = SPP_OSAL_GetEventGroupsBuffer();
    p_bmp->p_event_group = SPP_OSAL_EventGroupCreate(p_buffer_eg);

    p_bmp->isr_ctx.event_group = p_bmp->p_event_group;
    p_bmp->isr_ctx.bits        = BMP390_EVT_DRDY;

    /* --- GPIO INT + ISR (interna fija del HAL) --- */
    SPP_HAL_GPIO_ConfigInterrupt(p_bmp->int_pin, p_bmp->int_intr_type, p_bmp->int_pull);
    SPP_HAL_GPIO_RegisterISR(p_bmp->int_pin, (void*)&p_bmp->isr_ctx);

    /* Fin del init */
    SPP_OSAL_TaskDelete(NULL);
}


//--------------------CONFIG and CHECK---------------------------

retval_t bmp390_soft_reset(void *p_spi)
{
    spp_uint8_t buf[2] = 
    {
        (spp_uint8_t)BMP390_SOFT_RESET_REG,
        (spp_uint8_t)BMP390_SOFT_RESET_CMD
    };

    retval_t ret = SPP_HAL_SPI_Transmit(p_spi, buf, (spp_uint8_t)sizeof(buf));

    vTaskDelay(pdMS_TO_TICKS(100));

    return ret;
}

retval_t bmp390_enable_spi_mode(void *p_spi)
{
    spp_uint8_t buf[2] = 
    {
        (spp_uint8_t)BMP390_IF_CONF_REG,
        (spp_uint8_t)BMP390_IF_CONF_SPI
    };

    retval_t ret = SPP_HAL_SPI_Transmit(p_spi, buf, (spp_uint8_t)sizeof(buf));

    vTaskDelay(pdMS_TO_TICKS(100));

    return ret;
}

retval_t bmp390_config_check(void *p_spi)
{
    retval_t ret;

    spp_uint8_t buf[4] = 
    {
        (spp_uint8_t)(READ_OP | BMP390_IF_CONF_REG),    EMPTY_MESSAGE,
        (spp_uint8_t)(READ_OP | BMP390_SOFT_RESET_REG), EMPTY_MESSAGE
    };

    ret = SPP_HAL_SPI_Transmit(p_spi, buf, (spp_uint8_t)sizeof(buf));

    return ret;
}

//--------------------PREPARE READ---------------------------

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

    tcalib->t_lin = 0.0f;

    return SPP_OK;
}


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

    return SPP_OK;
}

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

    return SPP_OK;
}

float bmp390_compensate_temperature(spp_uint32_t raw_temp, bmp390_temp_params_t *params)
{
    float partial1 = (float)raw_temp - params->PAR_T1;
    float partial2 = partial1 * params->PAR_T2;
    float t_lin = partial2 + (partial1 * partial1) * params->PAR_T3;

    return t_lin;
}

//--------------------READ PRESS---------------------------

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

    return SPP_OK;
}

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

    return SPP_OK;
}

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

    return SPP_OK;
}

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

// //--------------------AUX FUNCTIONS (GENERAL)---------------------------

// esp_err_t bmp390_config(data_t *p_dev)
// {
//     ret = bmp390_soft_reset(p_dev);

//     ret = bmp390_enable_spi_mode(p_dev);

//     vTaskDelay(pdMS_TO_TICKS(50));
    
//     ret = bmp390_read_chip_id(p_dev, &id);

//     ret = bmp390_read_if_conf(p_dev, &ifc);

//     return ESP_OK;
// }//End BMP config

// void bmp390_prepare_mode(data_t *p_dev)
// {
//     ret = bmp390_set_mode_normal(p_dev);

//     ret = bmp390_set_osr_temp(p_dev);

//     ret = bmp390_set_odr(p_dev);

//     ret = bmp390_set_iir(p_dev);

//     vTaskDelay(pdMS_TO_TICKS(50)); // Dejar tiempo al primer dato en Normal mode

// }//End BMP prepare mode

// void bmp390_prepare_temp(data_t *p_dev)
// {
//     ret = bmp390_read_raw_temp_coeffs(p_dev, &raw_calib); //Raw coeffs
//     ESP_LOGI(TAG, "Coef raw T1=%u, T2=%d, T3=%d",
//                 raw_calib.par_t1,
//                 raw_calib.par_t2,
//                 raw_calib.par_t3);

    
//     ret = bmp390_calibrate_temp_params(p_dev, &temp_params); //Calib
//     ESP_LOGI(TAG, "PAR_T1 calibrado: %.4f",  temp_params.PAR_T1);
//     ESP_LOGI(TAG, "PAR_T2 calibrado: %.6e",  temp_params.PAR_T2);
//     ESP_LOGI(TAG, "PAR_T3 calibrado: %.6e",  temp_params.PAR_T3);

// }//End prepare temp

// void bmp390_prepare_press(data_t *p_dev)
// {
//     ret = bmp390_read_raw_press_coeffs(p_dev, &raw_press_calib); //Raw coeffs

//     ESP_LOGI(TAG, "Coef raw P1=%u, P2=%u, P3=%d, P4=%d, P5=%d, P6=%d, P7=%d, P8=%d, P9=%d, P10=%d, P11=%d",
//                 raw_press_calib.par_p1,
//                 raw_press_calib.par_p2,
//                 raw_press_calib.par_p3,
//                 raw_press_calib.par_p4,
//                 raw_press_calib.par_p5,
//                 raw_press_calib.par_p6,
//                 raw_press_calib.par_p7,
//                 raw_press_calib.par_p8,
//                 raw_press_calib.par_p9,
//                 raw_press_calib.par_p10,
//                 raw_press_calib.par_p11);

//     ret = bmp390_calibrate_press_params(p_dev, &press_params); //Calib
//     ESP_LOGI(TAG, "PAR_P1 calibrado: %.6f", press_params.PAR_P1);
//     ESP_LOGI(TAG, "PAR_P2 calibrado: %.6f", press_params.PAR_P2);
//     ESP_LOGI(TAG, "PAR_P3 calibrado: %.6f", press_params.PAR_P3);
//     ESP_LOGI(TAG, "PAR_P4 calibrado: %.6f", press_params.PAR_P4);
//     ESP_LOGI(TAG, "PAR_P5 calibrado: %.6f", press_params.PAR_P5);
//     ESP_LOGI(TAG, "PAR_P6 calibrado: %.6f", press_params.PAR_P6);
//     ESP_LOGI(TAG, "PAR_P7 calibrado: %.6f", press_params.PAR_P7);
//     ESP_LOGI(TAG, "PAR_P8 calibrado: %.6f", press_params.PAR_P8);
//     ESP_LOGI(TAG, "PAR_P9 calibrado: %.6f", press_params.PAR_P9);
//     ESP_LOGI(TAG, "PAR_P10 calibrado: %.6f", press_params.PAR_P10);
//     ESP_LOGI(TAG, "PAR_P11 calibrado: %.6f", press_params.PAR_P11);

// }//End prepare press

// esp_err_t bmp390_prepare_read(data_t *p_dev)
// {
//     bmp390_prepare_mode(p_dev); 

//     bmp390_prepare_temp(p_dev); 

//     bmp390_prepare_press(p_dev);

//     return ESP_OK;
// }


// esp_err_t bmp390_read_temp(data_t *p_dev)
// {
//     ret = bmp390_wait_temp_ready(p_dev);

//     ret = bmp390_read_raw_temp(p_dev, &raw_temp);
//     ESP_LOGI(TAG, "Raw temp: %u", raw_temp);

//     float comp_temp = bmp390_compensate_temperature(raw_temp, &temp_params);
//     ESP_LOGI(TAG, "Temp compensada: %.2f °C", comp_temp);

//     return ESP_OK;
// }//End read temp

// esp_err_t bmp390_calc_altitude(data_t *p_dev)
// {
//     ret = bmp390_wait_press_ready(p_dev);


//     ret = bmp390_read_raw_press(p_dev, &raw_press);

//     ESP_LOGI(TAG, "Raw press: %u", raw_press);

//     t_lin = bmp390_compensate_temperature(raw_temp, &temp_params);
//     float p_pa = bmp390_compensate_pressure(raw_press, t_lin, &press_params);
//     ESP_LOGI(TAG, "Presión comp.: %.2f Pa", p_pa);

//     float altitude = 44330.0f * (1.0f - powf(p_pa/101325.0f, 1.0f/5.255f));
//     ESP_LOGI(TAG, "Altura: %.2f m", altitude);

//     // Delay para pruebas (5 s)
//     vTaskDelay(pdMS_TO_TICKS(5000));

//     return ESP_OK;
// }//End calc altitude


// esp_err_t bmp390_read_measurements(data_t *p_dev)
// {
//     ret = bmp390_read_temp(p_dev); //Temp
//     if (ret != ESP_OK) 
//     {
//         ESP_LOGE(TAG, "Error read temp BMP390: %d", ret);

//     }

//     ret = bmp390_calc_altitude(p_dev); //Press and Alt
//     if (ret != ESP_OK) 
//     {
//         ESP_LOGE(TAG, "Error calc altitude BMP390: %d", ret);  
//     }

//     return ESP_OK;
// }//End BMP read