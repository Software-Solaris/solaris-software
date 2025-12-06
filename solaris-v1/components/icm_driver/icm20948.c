#include "icm20948.h"
#include "driver/spi_common.h"
#include "spi.h"

// static const char* TAG = "ICM20948";

retval_t IcmInit(void *p_data) {
    icm_data_t *p_data_icm = (icm_data_t*)p_data;
    retval_t ret = SPP_ERROR;
    void* p_handler_spi;

    ret = SPP_HAL_SPI_BusInit(); /** Already done in the BMP who is initialized first */
    if (ret != SPP_OK) return ret;

    p_handler_spi = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_handler_spi);
    if (ret != SPP_OK) return ret;

    p_data_icm->p_handler_spi = (void*)p_handler_spi;
    return SPP_OK;
}

retval_t IcmConfig(void *p_data)
{
    icm_data_t *p_data_icm = (icm_data_t*)p_data;
    retval_t ret;
    spp_uint8_t data[2];

    /** 1) Reset of ICM: writing 0x80 on PWR_MGMT_1 */
    data[0] = WRITE_OP | REG_PWR_MGMT_1;
    data[1] = BIT_H_RESET;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    /** 2) Wake up + temp disable (igual que el fichero bueno): writing 0x09 on PWR_MGMT_1 */
    data[0] = WRITE_OP | REG_PWR_MGMT_1;
    data[1] = 0x09;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(20));

    /** 3) WHO_AM_I read */
    data[0] = READ_OP | REG_WHO_AM_I;
    data[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** 4) Enable ICM resources: USER_CTRL = 0x30 */
    data[0] = WRITE_OP | REG_USER_CTRL;
    data[1] = USER_CTRL_CONFIG;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** (Para ser idéntico al "bueno": lo escribe 2 veces) */
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** 5) Swap to bank 3 */
    data[0] = WRITE_OP | REG_BANK_SEL;
    data[1] = 0x30;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** 6) Internal I2C transaction speed: I2C_CTRL = 0x07 */
    data[0] = WRITE_OP | REG_I2C_CTRL;
    data[1] = I2C_SP_CONFIG;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** -------- MAGNETOMETER: leer WHO_AM_I vía SLV4 (igual que el fichero bueno) -------- */

    /** SLV4_ADDR = MAGNETO_RD_ADDR (0x8C) */
    data[0] = WRITE_OP | REG_SLV4_ADDR;
    data[1] = MAGNETO_RD_ADDR;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** SLV4_REG = MAGNETO_WHO_AM_I (0x01) */
    data[0] = WRITE_OP | REG_SLV4_REG;
    data[1] = MAGNETO_WHO_AM_I;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** SLV4_CTRL = 0x80 (dispara lectura) */
    data[0] = WRITE_OP | REG_SLV4_CTRL;
    data[1] = MAGNETO_CONFIG_1;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    /** SLV4_DI read (resultado WHO_AM_I del magnetómetro) */
    data[0] = READ_OP | REG_SLV4_DI;
    data[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** -------- MAGNETOMETER: activar modo 2 vía SLV4 (igual que el fichero bueno) -------- */

    /** SLV4_ADDR = MAGNETO_WR_ADDR (0x0C) */
    data[0] = WRITE_OP | REG_SLV4_ADDR;
    data[1] = MAGNETO_WR_ADDR;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** SLV4_REG = MAGNETO_CTRL_2 (0x31) */
    data[0] = WRITE_OP | REG_SLV4_REG;
    data[1] = MAGNETO_CTRL_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** SLV4_DO = MAGNETO_MSM_MODE_2 (0x04) */
    data[0] = WRITE_OP | REG_SLV4_DO;
    data[1] = MAGNETO_MSM_MODE_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** SLV4_CTRL = 0x80 */
    data[0] = WRITE_OP | REG_SLV4_CTRL;
    data[1] = MAGNETO_CONFIG_1;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    /** -------- MAGNETOMETER: lecturas periódicas vía SLV0 (igual que el fichero bueno) -------- */

    /** SLV0_ADDR = MAGNETO_RD_ADDR (0x8C) */
    data[0] = WRITE_OP | REG_SLV0_ADDR;
    data[1] = MAGNETO_RD_ADDR;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** SLV0_REG = MAGNETO_START_RD (0x10) */
    data[0] = WRITE_OP | REG_SLV0_REG;
    data[1] = MAGNETO_START_RD;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** SLV0_CTRL = MAGNETO_CONFIG_2 (0x89) */
    data[0] = WRITE_OP | REG_SLV0_CTRL;
    data[1] = MAGNETO_CONFIG_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /** Back to bank 0 */
    data[0] = WRITE_OP | REG_BANK_SEL;
    data[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

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
