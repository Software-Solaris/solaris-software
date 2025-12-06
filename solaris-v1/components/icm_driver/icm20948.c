#include "icm20948.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "spi.h"
#include "task.c"

static const char* TAG = "ICM20948"; 
static esp_err_t ret;

esp_err_t send_message(data_t *p_dev, uint8_t tx[2], uint8_t rx[2]) {
    // Ajuste de parámetros de transacción
    p_dev->trans_desc.length = 16;
    p_dev->trans_desc.tx_buffer = tx;
    p_dev->trans_desc.rx_buffer = rx;

    esp_err_t trans_result = spi_device_transmit(p_dev->handle, &p_dev->trans_desc);
    return trans_result;
}

int16_t get_raw_axis_data(data_t *p_dev, uint8_t h_reg, uint8_t l_reg) {

    uint8_t tx_h[2] = { (uint8_t)(READ_OP | h_reg), EMPTY_MESSAGE };
    uint8_t rx_h[2] = { 0 };
    ret = send_message(p_dev, tx_h, rx_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading high byte from reg 0x%02X", h_reg);
        return 0;
    }

    uint8_t tx_l[2] = { (uint8_t)(READ_OP | l_reg), EMPTY_MESSAGE };
    uint8_t rx_l[2] = { 0 };
    ret = send_message(p_dev, tx_l, rx_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading low byte from reg 0x%02X", l_reg);
        return 0;
    }

    return (int16_t)((rx_h[1] << 8) | rx_l[1]);
}


retval_t IcmInit(void *p_data) {
    icm_data_t *p_data_icm = (icm_data_t*)p_data;
    retval_t ret = SPP_ERROR;
    void* p_handler_spi;

    ret = SPP_HAL_SPI_BusInit(); /** Already done in the BMP who is initialized first */
    p_handler_spi = SPP_HAL_SPI_GetHandler();
    spi_device_handle_t *p_handle = (spi_device_handle_t*)p_handler_spi;
    ret = SPP_HAL_SPI_DeviceInit(p_handler_spi);
    // p_data_icm->p_handler_spi = (void*)p_handler_spi;
    return SPP_OK;
}


retval_t IcmConfig(void *p_data) {
        icm_data_t *p_data_icm = (icm_data_t*)p_data;
    
        // Reset del sensor: escribir 0x80 en PWR_MGMT_1
        spp_uint8_t data[14] = { (spp_uint8_t) (WRITE_OP | REG_PWR_MGMT_1), BIT_H_RESET,     /** Reset the sensor */
                                (spp_uint8_t) (WRITE_OP | REG_PWR_MGMT_1), 0x01,            /** Take sensor out of sleep mode  */
                                (spp_uint8_t) (READ_OP | REG_WHO_AM_I), EMPTY_MESSAGE,      /** Read WHO_AM_I register */
                                (spp_uint8_t) (WRITE_OP | REG_LP_CONFIG), I2C_DM_DEAC,      /** Deactivate I2C_DUTY_CYCLED */
                                (spp_uint8_t) (WRITE_OP | REG_USER_CTRL), USER_CTRL_CONFIG, /** Enable resources of ICM - FIFO, internal I2C,...*/
                                (spp_uint8_t) (WRITE_OP | REG_BANK_SEL), 0x30,              /** Change to BANK 3 of registers */
                                (spp_uint8_t) (WRITE_OP | REG_I2C_CTRL), I2C_SP_CONFIG,     /** Define the internal I2C speed transaction  */                       
                            };
    
        retval_t ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, sizeof(data)/sizeof(data[0]));
    
        ESP_LOGI(TAG, "WHO_AM_I register has 0x%02X | should be: 0xEA", data[5]);

        return SPP_OK;

}
    // /* CONFIGURACIÓN DEL MAGNETÓMETRO

    //     1. Hacer un reset y sacarlo del power-mode desde el SLV_4
    //         1.1 Acceder al magnetómetro con dirección física en modo escritura: escribir 0x0C en I2C_SLV4_ADDR
    //         1.2 Dar dirección de "Control 2": escribir 0x31 en I2C_SLV4_REG
    //         1.3 Escribir mensaje a enviarle: escribir 0x08 (mode 2) en I2C_SLV4_DO
    //         1.4 Configurar los datos de transacción: escribir 0x81 en I2C_SLV4_CTRL
    //      2. Configurar las lecturas periódicas de los datos desde el SLV_0
    //         2.1 Acceder al magnetómetro con dirección física en modo lectura: escribir 0x8C en I2C_SLV0_ADDR
    //         2.2 Dar dirección del principio de los registros: escribir 0x11 en I2C_SLV0_REG
    //         2.3 Configurar los datos de transacción: escribir 0x81 en I2C_SLV0_CTRL
    // */

    // // 1.1
    // uint8_t tx_magneto[2] = { (uint8_t) (WRITE_OP | REG_SLV4_ADDR), MAGNETO_WR_ADDR };
    // uint8_t rx_magneto[2] = { 0, 0 };

    // ret = send_message(p_dev, tx_magneto, rx_magneto);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Error on SLV4_ADDR configuration");
    // }

    // // 1.2
    // uint8_t tx_magneto_read[2] = { (uint8_t) (WRITE_OP | REG_SLV4_REG), MAGNETO_CTRL_2 };
    // uint8_t rx_magneto_read[2] = { 0, 0 };

    // ret = send_message(p_dev, tx_magneto_read, rx_magneto_read);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Error on SLV4_REG configuration");
    // }

    // // 1.3
    // uint8_t tx_magneto_msg[2] = { (uint8_t) (WRITE_OP | REG_SLV4_DO), MAGNETO_MSM_MODE_2 };
    // uint8_t rx_magneto_msg[2] = { 0, 0 };

    // ret = send_message(p_dev, tx_magneto_msg, rx_magneto_msg);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Error on SLV4_DO configuration");
    // }

    // // 1.4
    // uint8_t tx_magneto_ctrl[2] = { (uint8_t) (WRITE_OP | REG_SLV4_CTRL), MAGNETO_CONFIG_1 };
    // uint8_t rx_magneto_ctrl[2] = { 0, 0 };

    // ret = send_message(p_dev, tx_magneto_ctrl, rx_magneto_ctrl);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Error on SLV4_ctrl configuration");
    // }
    // vTaskDelay(pdMS_TO_TICKS(100)); // Delay para llevar a cabo la escritura


    // // 2.1
    // uint8_t tx_magneto_1[2] = { (uint8_t) (WRITE_OP | REG_SLV0_ADDR), MAGNETO_RD_ADDR };
    // uint8_t rx_magneto_1[2] = { 0, 0 };

    // ret = send_message(p_dev, tx_magneto_1, rx_magneto_1);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Error on SLV0_ADDR configuration (2.1)");
    // }

    // // 2.2
    // uint8_t tx_magneto_read_1[2] = { (uint8_t) (WRITE_OP | REG_SLV0_REG), MAGNETO_START_RD };
    // uint8_t rx_magneto_read_1[2] = { 0, 0 };

    // ret = send_message(p_dev, tx_magneto_read_1, rx_magneto_read_1);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Error on SLV0_REG configuration (2.2)");
    // }

    // // 2.3
    // uint8_t tx_magneto_ctrl_1[2] = { (uint8_t) (WRITE_OP | REG_SLV0_CTRL), MAGNETO_CONFIG_2 };
    // uint8_t rx_magneto_ctrl_1[2] = { 0, 0 };

    // ret = send_message(p_dev, tx_magneto_ctrl_1, rx_magneto_ctrl_1);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Error on SLV0_CTRL configuration (2.3)");
    // }

    // // Devolver a banco 0 de registros
    // uint8_t tx_bank_sel_2[2] = { (uint8_t) (WRITE_OP | REG_BANK_SEL), 0x00 };
    // uint8_t rx_bank_sel_2[2] = { 0, 0 };

    // ret = send_message(p_dev, tx_bank_sel_2, rx_bank_sel_2);
// }

esp_err_t icm20948_prepare_read(data_t *p_dev) {

    // Cambiar a banco 2 de registros
    uint8_t tx_bank_sel_3[2] = { (uint8_t) (WRITE_OP | REG_BANK_SEL), 0x20 };
    uint8_t rx_bank_sel_3[2] = { 0, 0 };

    ret = send_message(p_dev, tx_bank_sel_3, rx_bank_sel_3);

    // Configuración del filtro de paso bajo del acelerómetro: escribir la configuración deseada en ACCEL_CONFIG
    uint8_t tx_accel_conf[2] = { (uint8_t) (WRITE_OP | REG_ACCEL_CONFIG), ACCEL_FILTER_SELEC };
    uint8_t rx_accel_conf[2] = { 0, 0 };

    ret = send_message(p_dev, tx_accel_conf, rx_accel_conf);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "ACCEL_CONFIG modified succesfully, now low pass filter is activated");
    }

    // Configuración del filtro de paso bajo del giroscopio: escribir la configuración deseada en GYRO_CONFIG
    uint8_t tx_gyro_conf[2] = { (uint8_t) (WRITE_OP | REG_GYRO_CONFIG), GYRO_FILTER_SELEC };
    uint8_t rx_gyro_conf[2] = { 0, 0 };

    ret = send_message(p_dev, tx_gyro_conf, rx_gyro_conf);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "GYRO_CONFIG modified succesfully, now low pass filter is activated");
    }

    vTaskDelay(pdMS_TO_TICKS(500)); //Espera al modificar características de los sensores

    // Devolver a banco 0 para las lecturas de datos
    uint8_t tx_bank_sel_4[2] = { (uint8_t) (WRITE_OP | REG_BANK_SEL), 0x00 };
    uint8_t rx_bank_sel_4[2] = { 0, 0 };

    ret = send_message(p_dev, tx_bank_sel_4, rx_bank_sel_4);

    return ESP_OK;
}

esp_err_t icm20948_read_measurements(data_t *p_dev) {
    // Inicialización de variables
    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    float accel_x, accel_y, accel_z;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
    float gyro_x, gyro_y, gyro_z;
    int16_t magneto_x_raw, magneto_y_raw, magneto_z_raw;
    float magneto_x, magneto_y, magneto_z;

    // Variables de compensación (para la icm de Vladik, tomar medidas de calibración sin offset antes de su uso)
    float ax_offset = 1.622;
    float ay_offset = 0.288;
    float az_offset = -8.682;
    float gx_offset = -0.262;
    float gy_offset = -2.372;
    float gz_offset = 0.014;

    // Accel: X
    accel_x_raw = get_raw_axis_data(p_dev, REG_ACCEL_X_H, REG_ACCEL_X_L);
    accel_x = (((float)accel_x_raw / 16384.0f) * 9.80665f) + ax_offset; // Pasamos el valor digital -> g en función de la sensibilidad -> m/s2

    // Accel: Y
    accel_y_raw = get_raw_axis_data(p_dev, REG_ACCEL_Y_H, REG_ACCEL_Y_L);
    accel_y = (((float)accel_y_raw / 16384.0f) * 9.80665f) + ay_offset;

    // Accel: Z
    accel_z_raw = get_raw_axis_data(p_dev, REG_ACCEL_Z_H, REG_ACCEL_Z_L);
    accel_z = (((float)accel_z_raw / 16384.0f) * 9.80665f) + az_offset;

    // Gyro: X
    gyro_x_raw = get_raw_axis_data(p_dev, REG_GYRO_X_H, REG_GYRO_X_L);
    gyro_x = ((float)gyro_x_raw / 131.0f) + gx_offset; // Pasamos el valor digital -> dps (degrees per second)

    // Gyro: Y
    gyro_y_raw = get_raw_axis_data(p_dev, REG_GYRO_Y_H, REG_GYRO_Y_L);
    gyro_y = ((float)gyro_y_raw / 131.0f) + gy_offset;

    // Gyro: Z
    gyro_z_raw = get_raw_axis_data(p_dev, REG_GYRO_Z_H, REG_GYRO_Z_L);
    gyro_z = ((float)gyro_z_raw / 131.0f) + gz_offset;

    // Magneto: X
    magneto_x_raw = get_raw_axis_data(p_dev, REG_MAGNETO_X_H, REG_MAGNETO_X_L);
    magneto_x = ((float)magneto_x_raw * 0.15);

    // Logs
    ESP_LOGI(TAG, "Accel g    - X: %.2f, Y: %.2f, Z: %.2f", accel_x, accel_y, accel_z);
    ESP_LOGI(TAG, "Gyro dps   - X: %.2f, Y: %.2f, Z: %.2f", gyro_x, gyro_y, gyro_z);
    ESP_LOGI(TAG, "DATOS EN CRUDO DEL MAGNETÓMETRO EN X CRUDOS (PRUEBA): %d", magneto_x_raw);
    ESP_LOGI(TAG, "DATOS MAGNETO_X PROCESADOS (PRUEBA): %f", magneto_x);


    return ESP_OK;
}