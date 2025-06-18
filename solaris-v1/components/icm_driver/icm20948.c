#include "icm20948.h"
#include "esp_log.h"
#include "driver/spi_common.h"

static const char* TAG = "ICM20948"; 

esp_err_t send_message(data_t *p_dev, uint8_t tx[2], uint8_t rx[2]) {
    // Ajuste de parámetros de transacción
    p_dev->trans_desc.length = 16;
    p_dev->trans_desc.tx_buffer = tx;
    p_dev->trans_desc.rx_buffer = rx;

    esp_err_t trans_result = spi_device_transmit(p_dev->handle, &p_dev->trans_desc);
    return trans_result;
}

int16_t get_raw_axis_data(data_t *p_dev, uint8_t h_reg, uint8_t l_reg) {
    esp_err_t ret;

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


esp_err_t icm20948_init(data_t *p_dev) {

    // 1. Inicializa la configuración del bus SPI
    p_dev->buscfg.miso_io_num = PIN_NUM_CIPO;
    p_dev->buscfg.mosi_io_num = PIN_NUM_COPI;
    p_dev->buscfg.sclk_io_num = PIN_NUM_CLK;
    p_dev->buscfg.quadwp_io_num = -1;
    p_dev->buscfg.quadhd_io_num = -1;
    p_dev->buscfg.max_transfer_sz = 4096;

    esp_err_t ret = spi_bus_initialize(SPI_HOST_USED, &p_dev->buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error spi_bus_initialize: %d", ret);
        return ret;
    } 
    ESP_LOGI(TAG, "SPI Bus initialized.");

    // 2. Configura el dispositivo SPI (CS, velocidad, modo, etc.)
    p_dev->devcfg.clock_speed_hz = 100000;
    p_dev->devcfg.mode = 3;           // Probar en modo 0 si falla
    p_dev->devcfg.spics_io_num = PIN_NUM_CS;
    p_dev->devcfg.queue_size = 1;
    p_dev->devcfg.address_bits = 0;
    p_dev->devcfg.command_bits = 0;
    p_dev->devcfg.dummy_bits = 0;
    p_dev->devcfg.flags = 0;
    p_dev->devcfg.duty_cycle_pos = 128;
    p_dev->devcfg.pre_cb = NULL;
    p_dev->devcfg.post_cb = NULL;
    vTaskDelay(pdMS_TO_TICKS(100)); // Espera adicional de 100 ms

    ret = spi_bus_add_device(SPI_HOST_USED, &p_dev->devcfg, &p_dev->handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error spi_bus_add_device: %d", ret);
        return ret;
    } 
    ESP_LOGI(TAG, "Device added to SPI bus.");



    // Reset del sensor: escribir 0x80 en PWR_MGMT_1
    uint8_t tx_reset[2] = { (uint8_t) (WRITE_OP | REG_PWR_MGMT_1), BIT_H_RESET };
    uint8_t rx_reset[2] = { 0, 0 };

    ret = send_message(p_dev, tx_reset, rx_reset);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor reset returned an error: %d", ret);
        return ret;
    }

    // Lectura del contenido del WHO_AM_i: leer 0x00
    uint8_t tx_who_am_i[2] = { (uint8_t) (READ_OP | REG_WHO_AM_I), EMPTY_MESSAGE };
    uint8_t rx_who_am_i[2] = { 0, 0 };

    ret = send_message(p_dev, tx_who_am_i, rx_who_am_i);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I reading failed: %d", ret);
        return ret;
    } else {
        p_dev->sensor_id = rx_who_am_i[1]; // Datos útiles en el segundo bit
        ESP_LOGI(TAG, "WHO_AM_I register has 0x%02X | should be: 0xEA", p_dev->sensor_id);
    }

    return ret;
}

esp_err_t icm20948_config(data_t *p_dev) {
    esp_err_t ret;

    // Desactivar el modo "I2C_DUTY_CYCLED": escribir 0x00 en LP_CONFIG
    uint8_t tx_lp_config[2] = { (uint8_t) (WRITE_OP | REG_LP_CONFIG), I2C_DEAC };
    uint8_t rx_lp_config[2] = { 0, 0 };

    ret = send_message(p_dev, tx_lp_config, rx_lp_config);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "LP_CONFIG changed,  read 0x%02X | should be: 0x00", rx_lp_config[1]);
    }

    // Sacar el sensor del modo "SLEEP_MODE": escribir 0x01 en PWR_MGMT_1
    uint8_t tx_sleep_off[2] = { (uint8_t) (WRITE_OP | REG_PWR_MGMT_1), 0x01 };
    uint8_t rx_sleep_off[2] = { 0, 0 };
    vTaskDelay(pdMS_TO_TICKS(20));
    ret = send_message(p_dev, tx_sleep_off, rx_sleep_off);

    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "PWR_MGMT_1 changed, read 0x%02X | should be: 0x01", rx_sleep_off[1]);
    }

    /* 
    // Cambiar a banco 2 de registros
    uint8_t tx_bank_sel[2] = { (uint8_t) (WRITE_OP | REG_BANK_SEL_0), 0x02 };
    uint8_t rx_bank_sel[2] = { 0, 0 };

    ret = send_message(p_dev, tx_bank_sel, rx_bank_sel);

    // Leer datos de ACCEL_CONFIG y ACCEL_CONFIG_2
    uint8_t tx_accel_conf_1[2] = { (uint8_t) (READ_OP) | 0x14, EMPTY_MESSAGE };
    uint8_t rx_accel_conf_1[2] = { 0, 0 };

    ret = send_message(p_dev, tx_accel_conf_1, rx_accel_conf_1);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "ACCEL_CONFIG read 0x%02X", rx_accel_conf_1[1]);
    }

    uint8_t tx_accel_conf_2[2] = { (uint8_t) (READ_OP | 0x15), EMPTY_MESSAGE };
    uint8_t rx_accel_conf_2[2] = { 0, 0 };

    ret = send_message(p_dev, tx_accel_conf_2, rx_accel_conf_2);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "PWR_MGMT_1 changed, read 0x%02X", rx_accel_conf_2[1]);
    } */

    return ESP_OK;
}

esp_err_t icm20948_get_measurements(data_t *p_dev) {
    // Inicialización de variables
    esp_err_t ret = ESP_OK;
    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    float accel_x, accel_y, accel_z;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
    float gyro_x, gyro_y, gyro_z;

    // Accel: X
    accel_x_raw = get_raw_axis_data(p_dev, REG_ACCEL_X_H, REG_ACCEL_X_L);
    accel_x = 2.03f + ((float)accel_x_raw / 16384.0f) * 9.80665f;

    // Accel: Y
    accel_y_raw = get_raw_axis_data(p_dev, REG_ACCEL_Y_H, REG_ACCEL_Y_L);
    accel_y = 2.03f + ((float)accel_y_raw / 16384.0f) * 9.80665f;

    // Accel: Z
    accel_z_raw = get_raw_axis_data(p_dev, REG_ACCEL_Z_H, REG_ACCEL_Z_L);
    accel_z = 2.03f + ((float)accel_z_raw / 16384.0f) * 9.80665f;

    // Gyro: X
    gyro_x_raw = get_raw_axis_data(p_dev, REG_GYRO_X_H, REG_GYRO_X_L);
    gyro_x = ((float)gyro_x_raw / 131.0f);  // para ±250 dps

    // Gyro: Y
    gyro_y_raw = get_raw_axis_data(p_dev, REG_GYRO_Y_H, REG_GYRO_Y_L);
    gyro_y = ((float)gyro_y_raw / 131.0f);

    // Gyro: Z
    gyro_z_raw = get_raw_axis_data(p_dev, REG_GYRO_Z_H, REG_GYRO_Z_L);
    gyro_z = ((float)gyro_z_raw / 131.0f);

    // ---------- Logs ----------
    ESP_LOGI(TAG, "Accel (m/s²) - X: %.2f, Y: %.2f, Z: %.2f", accel_x, accel_y, accel_z);
    ESP_LOGI(TAG, "Gyro  (°/s)  - X: %.2f, Y: %.2f, Z: %.2f", gyro_x, gyro_y, gyro_z);

    return ret;
}