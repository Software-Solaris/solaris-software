#include "icm20948.h"
#include "driver/spi_common.h"
#include "spi.h"

// static const char* TAG = "ICM20948"; 

retval_t IcmInit(void *p_data) {
    icm_data_t *p_data_icm = (icm_data_t*)p_data;
    retval_t ret = SPP_ERROR;
    void* p_handler_spi;

    ret = SPP_HAL_SPI_BusInit(); /** Already done in the BMP who is initialized first */
    p_handler_spi = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_handler_spi);
    p_data_icm->p_handler_spi = (void*)p_handler_spi;
    return SPP_OK;
}


retval_t IcmConfig(void *p_data)
{
    icm_data_t *p_data_icm = (icm_data_t*)p_data;
    retval_t ret;

    /** Sensor reset */
    {
        spp_uint8_t data[2] = { 
                                (spp_uint8_t)(WRITE_OP | REG_PWR_MGMT_1), 
                                BIT_H_RESET
                              };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi,
                                   data,
                                   sizeof(data)/sizeof(data[0]));
        if (ret != SPP_OK) return ret;
    }

    /** Exit sleep mode */
    {
        spp_uint8_t data[2] = { 
                                (spp_uint8_t)(WRITE_OP | REG_PWR_MGMT_1), 
                                0x01
                              };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi,
                                   data,
                                   sizeof(data)/sizeof(data[0]));
        if (ret != SPP_OK) return ret;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /** Read WHO_AM_I register */
    {
        spp_uint8_t data[2] = { 
                                (spp_uint8_t)(READ_OP | REG_WHO_AM_I), 
                                EMPTY_MESSAGE
                              };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi,
                                   data,
                                   sizeof(data)/sizeof(data[0]));
        if (ret != SPP_OK) return ret;

        // ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expected: 0xEA)", data[1]);
    }

    /** General ICM configuration (single transaction) */
    {
        spp_uint8_t data[8] = {
            (spp_uint8_t)(WRITE_OP | REG_LP_CONFIG),  I2C_DM_DEAC,
            (spp_uint8_t)(WRITE_OP | REG_USER_CTRL),  USER_CTRL_CONFIG,
            (spp_uint8_t)(WRITE_OP | REG_BANK_SEL),   0x30,
            (spp_uint8_t)(WRITE_OP | REG_I2C_CTRL),   I2C_SP_CONFIG
        };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi,
                                   data,
                                   sizeof(data)/sizeof(data[0]));
        if (ret != SPP_OK) return ret;
    }

    /** Complete magnetometer configuration (single transaction) */
    {
        spp_uint8_t data[14] = {
            (spp_uint8_t)(WRITE_OP | REG_SLV4_ADDR),  MAGNETO_WR_ADDR,     /** Magnetometer access (write) */
            (spp_uint8_t)(WRITE_OP | REG_SLV4_REG),   MAGNETO_CTRL_2,      /** Control 2 register */
            (spp_uint8_t)(WRITE_OP | REG_SLV4_DO),    MAGNETO_MSM_MODE_2,  /** Measurement mode 2 */
            (spp_uint8_t)(WRITE_OP | REG_SLV4_CTRL),  MAGNETO_CONFIG_1,    /** Execute SLV4 transaction */
            (spp_uint8_t)(WRITE_OP | REG_SLV0_ADDR),  MAGNETO_RD_ADDR,    /** Magnetometer address (read) */
            (spp_uint8_t)(WRITE_OP | REG_SLV0_REG),   MAGNETO_START_RD,   /** First register to read */
            (spp_uint8_t)(WRITE_OP | REG_SLV0_CTRL),  MAGNETO_CONFIG_2    /** Enable periodic readings */
        };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi,
                                   data,
                                   sizeof(data)/sizeof(data[0]));
        if (ret != SPP_OK) return ret;
    }

    /** Return to register bank 0 */
    {
        spp_uint8_t data[2] = {
                                (spp_uint8_t)(WRITE_OP | REG_BANK_SEL),
                                0x00
                              };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi,
                                   data,
                                   sizeof(data)/sizeof(data[0]));
        if (ret != SPP_OK) return ret;
    }

    return SPP_OK;
}


retval_t IcmPrepareRead(void *p_data)
{
    icm_data_t *p_data_icm = (icm_data_t*)p_data;
    retval_t ret;

    /** Switch to register bank 2 */
    {
        spp_uint8_t data[2] = {
                                (spp_uint8_t)(WRITE_OP | REG_BANK_SEL),
                                0x20
                              };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi,
                                   data,
                                   sizeof(data)/sizeof(data[0]));
        if (ret != SPP_OK) return ret;
    }

    /** Configure accelerometer low pass filter */
    {
        spp_uint8_t data[2] = {
                                (spp_uint8_t)(WRITE_OP | REG_ACCEL_CONFIG),
                                ACCEL_FILTER_SELEC
                              };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi,
                                   data,
                                   sizeof(data)/sizeof(data[0]));
        if (ret != SPP_OK) return ret;
    }

    /** Configure gyroscope low pass filter */
    {
        spp_uint8_t data[2] = {
                                (spp_uint8_t)(WRITE_OP | REG_GYRO_CONFIG),
                                GYRO_FILTER_SELEC
                              };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi,
                                   data,
                                   sizeof(data)/sizeof(data[0]));
        if (ret != SPP_OK) return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    /** Return to register bank 0 */
    {
        spp_uint8_t data[2] = {
                                (spp_uint8_t)(WRITE_OP | REG_BANK_SEL),
                                0x00
                              };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi,
                                   data,
                                   sizeof(data)/sizeof(data[0]));
        if (ret != SPP_OK) return ret;
    }

    return SPP_OK;
}

retval_t IcmReadSensors(void *p_data)
{
    icm_data_t *p_data_icm = (icm_data_t*)p_data;
    retval_t ret;

    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    float accel_x, accel_y, accel_z;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
    float gyro_x, gyro_y, gyro_z;
    int16_t magneto_x_raw, magneto_y_raw, magneto_z_raw;
    float magneto_x, magneto_y, magneto_z;

    float ax_offset = 1.622;
    float ay_offset = 0.288;
    float az_offset = -8.682;
    float gx_offset = -0.262;
    float gy_offset = -2.372;
    float gz_offset = 0.014;

    /** ACCEL X */
    {
        spp_uint8_t h[2] = { (spp_uint8_t)(READ_OP | REG_ACCEL_X_H), EMPTY_MESSAGE };
        spp_uint8_t l[2] = { (spp_uint8_t)(READ_OP | REG_ACCEL_X_L), EMPTY_MESSAGE };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, h, 2);
        if (ret != SPP_OK) return ret;

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, l, 2);
        if (ret != SPP_OK) return ret;

        accel_x_raw = (h[1] << 8) | l[1];
        accel_x = (((float)accel_x_raw / 16384.0f) * 9.80665f) + ax_offset;
    }

    /** ACCEL Y */
    {
        spp_uint8_t h[2] = { (spp_uint8_t)(READ_OP | REG_ACCEL_Y_H), EMPTY_MESSAGE };
        spp_uint8_t l[2] = { (spp_uint8_t)(READ_OP | REG_ACCEL_Y_L), EMPTY_MESSAGE };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, h, 2);
        if (ret != SPP_OK) return ret;

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, l, 2);
        if (ret != SPP_OK) return ret;

        accel_y_raw = (h[1] << 8) | l[1];
        accel_y = (((float)accel_y_raw / 16384.0f) * 9.80665f) + ay_offset;
    }

    /** ACCEL Z */
    {
        spp_uint8_t h[2] = { (spp_uint8_t)(READ_OP | REG_ACCEL_Z_H), EMPTY_MESSAGE };
        spp_uint8_t l[2] = { (spp_uint8_t)(READ_OP | REG_ACCEL_Z_L), EMPTY_MESSAGE };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, h, 2);
        if (ret != SPP_OK) return ret;

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, l, 2);
        if (ret != SPP_OK) return ret;

        accel_z_raw = (h[1] << 8) | l[1];
        accel_z = (((float)accel_z_raw / 16384.0f) * 9.80665f) + az_offset;
    }

    /** GYRO X */
    {
        spp_uint8_t h[2] = { (spp_uint8_t)(READ_OP | REG_GYRO_X_H), EMPTY_MESSAGE };
        spp_uint8_t l[2] = { (spp_uint8_t)(READ_OP | REG_GYRO_X_L), EMPTY_MESSAGE };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, h, 2);
        if (ret != SPP_OK) return ret;

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, l, 2);
        if (ret != SPP_OK) return ret;

        gyro_x_raw = (h[1] << 8) | l[1];
        gyro_x = ((float)gyro_x_raw / 131.0f) + gx_offset;
    }

    /** GYRO Y */
    {
        spp_uint8_t h[2] = { (spp_uint8_t)(READ_OP | REG_GYRO_Y_H), EMPTY_MESSAGE };
        spp_uint8_t l[2] = { (spp_uint8_t)(READ_OP | REG_GYRO_Y_L), EMPTY_MESSAGE };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, h, 2);
        if (ret != SPP_OK) return ret;

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, l, 2);
        if (ret != SPP_OK) return ret;

        gyro_y_raw = (h[1] << 8) | l[1];
        gyro_y = ((float)gyro_y_raw / 131.0f) + gy_offset;
    }

    /** GYRO Z */
    {
        spp_uint8_t h[2] = { (spp_uint8_t)(READ_OP | REG_GYRO_Z_H), EMPTY_MESSAGE };
        spp_uint8_t l[2] = { (spp_uint8_t)(READ_OP | REG_GYRO_Z_L), EMPTY_MESSAGE };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, h, 2);
        if (ret != SPP_OK) return ret;

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, l, 2);
        if (ret != SPP_OK) return ret;

        gyro_z_raw = (h[1] << 8) | l[1];
        gyro_z = ((float)gyro_z_raw / 131.0f) + gz_offset;
    }

    /** MAGNETO X */
    {
        spp_uint8_t h[2] = { (spp_uint8_t)(READ_OP | REG_MAGNETO_X_H), EMPTY_MESSAGE };
        spp_uint8_t l[2] = { (spp_uint8_t)(READ_OP | REG_MAGNETO_X_L), EMPTY_MESSAGE };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, h, 2);
        if (ret != SPP_OK) return ret;

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, l, 2);
        if (ret != SPP_OK) return ret;

        magneto_x_raw = (h[1] << 8) | l[1];
        magneto_x = ((float)magneto_x_raw * 0.15f);
    }

    /** MAGNETO Y */
    {
        spp_uint8_t h[2] = { (spp_uint8_t)(READ_OP | REG_MAGNETO_Y_H), EMPTY_MESSAGE };
        spp_uint8_t l[2] = { (spp_uint8_t)(READ_OP | REG_MAGNETO_Y_L), EMPTY_MESSAGE };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, h, 2);
        if (ret != SPP_OK) return ret;

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, l, 2);
        if (ret != SPP_OK) return ret;

        magneto_y_raw = (h[1] << 8) | l[1];
        magneto_y = ((float)magneto_y_raw * 0.15f);
    }

    /** MAGNETO Z */
    {
        spp_uint8_t h[2] = { (spp_uint8_t)(READ_OP | REG_MAGNETO_Z_H), EMPTY_MESSAGE };
        spp_uint8_t l[2] = { (spp_uint8_t)(READ_OP | REG_MAGNETO_Z_L), EMPTY_MESSAGE };

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, h, 2);
        if (ret != SPP_OK) return ret;

        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, l, 2);
        if (ret != SPP_OK) return ret;

        magneto_z_raw = (h[1] << 8) | l[1];
        magneto_z = ((float)magneto_z_raw * 0.15f);
    }

    /** Logs */
    // ESP_LOGI(TAG, "Accel (g)    - X: %.2f, Y: %.2f, Z: %.2f", accel_x, accel_y, accel_z);
    // ESP_LOGI(TAG, "Gyro (dps)   - X: %.2f, Y: %.2f, Z: %.2f", gyro_x, gyro_y, gyro_z);
    // ESP_LOGI(TAG, "Magneto (uT) - X: %.2f, Y: %.2f, Z: %.2f", magneto_x, magneto_y, magneto_z);

    return SPP_OK;
}

void IcmGetSensorsData(void * p_data){
    for(;;){
        IcmReadSensors(p_data);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
