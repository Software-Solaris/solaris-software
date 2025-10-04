#include "icm20948.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "spi.h"

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
        ESP_LOGE(TAG, "Error spi_bus_initialize on ICM20948: %d", ret);
        return ret;
    } 
    ESP_LOGI(TAG, "SPI bus initialized on ICM20948");

    // 2. Configura el dispositivo SPI (CS, velocidad, modo, etc.)
    p_dev->devcfg.clock_speed_hz = 100000;
    p_dev->devcfg.mode = 3;           // Probar en modo 0 si falla
    p_dev->devcfg.spics_io_num = PIN_NUM_CS;
    p_dev->devcfg.queue_size = 20; // To execute queue SPI transaction
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
        ESP_LOGE(TAG, "Error spi_bus_add_device on ICM20948: %d", ret);
        return ret;
    } 
    ESP_LOGI(TAG, "ICM20948 added to SPI bus.");

    return ESP_OK;
}


esp_err_t icm20948_config(data_t *p_dev) {

    // Reset del sensor: escribir 0x80 en PWR_MGMT_1
    uint8_t tx_reset[2] = { (uint8_t) (WRITE_OP | REG_PWR_MGMT_1), BIT_H_RESET };
    uint8_t rx_reset[2] = { 0, 0 };

    ret = send_message(p_dev, tx_reset, rx_reset);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor reset returned an error: %d", ret);
        return ret;
    }

    // Sacar el sensor del modo "SLEEP_MODE" y desactivar sensor de temperatura: escribir 0x09 en PWR_MGMT_1
    uint8_t tx_sleep_off[2] = { (uint8_t) (WRITE_OP | REG_PWR_MGMT_1), 0x09 };
    uint8_t rx_sleep_off[2] = { 0, 0 };
    vTaskDelay(pdMS_TO_TICKS(20));
    ret = send_message(p_dev, tx_sleep_off, rx_sleep_off);

    // ?? You are reading garbage here
    // if (ret == ESP_OK) {
    //     ESP_LOGE(TAG, "PWR_MGMT_1 changed, read 0x%02X | should be: 0x09", rx_sleep_off[1]);
    // }

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

    // Habilitar recursos a emplear del icm: activar FIFO, I2C interno y DMP en USER_CTRL
    uint8_t tx_user_ctrl[2] = { (uint8_t) (WRITE_OP | REG_USER_CTRL), USER_CTRL_CONFIG };
    uint8_t rx_user_ctrl[2] = { 0, 0 };

    ret = send_message(p_dev, tx_user_ctrl, rx_user_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error on USER_CTRL sub-modules activation");
    }

    // Leemos le valor para asegurarnos
    uint8_t tx_user_ctrl1[2] = { (uint8_t) (READ_OP | REG_PWR_MGMT_1), EMPTY_MESSAGE };
    uint8_t rx_user_ctrl1[2] = { 0, 0 };

    ret = send_message(p_dev, tx_user_ctrl1, rx_user_ctrl1);

    // Cambiar a banco 3 de registros
    uint8_t tx_bank_sel[2] = { (uint8_t) (WRITE_OP | REG_BANK_SEL), 0x30 };
    uint8_t rx_bank_sel[2] = { 0, 0 };

    ret = send_message(p_dev, tx_bank_sel, rx_bank_sel);

    // Dejamos I2C_MST_ODR_CONFIG a 0x00 para que tome muestras a la velocidad del giroscopio
    // Definimos la velocidad de transacción I2C interno: escribir 0x07 en I2C_MST_CTRL
    uint8_t tx_i2c_ctrl[2] = { (uint8_t) (WRITE_OP | REG_I2C_CTRL), I2C_SP_CONFIG };
    uint8_t rx_i2c_ctrl[2] = { 0, 0 };

    ret = send_message(p_dev, tx_i2c_ctrl, rx_i2c_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error on i2c transmition configuration");
    }

    /* CONFIGURACIÓN DEL MAGNETÓMETRO

        1. Hacer un reset y sacarlo del power-mode desde el SLV_4
            1.1 Acceder al magnetómetro con dirección física en modo escritura: escribir 0x0C en I2C_SLV4_ADDR
            1.2 Dar dirección de "Control 2": escribir 0x31 en I2C_SLV4_REG
            1.3 Escribir mensaje a enviarle: escribir 0x08 (mode 2) en I2C_SLV4_DO
            1.4 Configurar los datos de transacción: escribir 0x81 en I2C_SLV4_CTRL
         2. Configurar las lecturas periódicas de los datos desde el SLV_0
            2.1 Acceder al magnetómetro con dirección física en modo lectura: escribir 0x8C en I2C_SLV0_ADDR
            2.2 Dar dirección del principio de los registros: escribir 0x11 en I2C_SLV0_REG
            2.3 Configurar los datos de transacción: escribir 0x81 en I2C_SLV0_CTRL
    */

    // Lectura del Device ID (usamos un scope en C con {})
    {
        // 1.1
        uint8_t tx_magneto[2] = { (uint8_t) (WRITE_OP | REG_SLV4_ADDR), (READ_OP | MAGNETO_WR_ADDR )};
        uint8_t rx_magneto[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto, rx_magneto);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_ADDR configuration");
        }

        // 1.2
        uint8_t tx_magneto_read[2] = { (uint8_t) (WRITE_OP | REG_SLV4_REG), MAGNETO_WHO_AM_I };
        uint8_t rx_magneto_read[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_read, rx_magneto_read);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_REG configuration");
        }

        // 1.4 - Indicamos que ejecute la transacción
        uint8_t tx_magneto_ctrl[2] = { (uint8_t) (WRITE_OP | REG_SLV4_CTRL), MAGNETO_CONFIG_1 };
        uint8_t rx_magneto_ctrl[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_ctrl, rx_magneto_ctrl);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_ctrl configuration");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay para llevar a cabo la escritura

        // Leemos el registro del WHO_AM_I del magneto
        uint8_t tx_magneto_read_1[2] = { (uint8_t) (READ_OP| REG_SLV4_DI), EMPTY_MESSAGE };
        uint8_t rx_magneto_read_1[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_read_1, rx_magneto_read_1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV0_REG configuration (2.2)");
        }
    } //End of scope

    // Activamos el magnetómetro
    {
        // 1.1
        uint8_t tx_magneto[2] = { (uint8_t) (WRITE_OP | REG_SLV4_ADDR), (WRITE_OP | MAGNETO_WR_ADDR )};
        uint8_t rx_magneto[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto, rx_magneto);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_ADDR configuration");
        }

        // 1.2
        uint8_t tx_magneto_read[2] = { (uint8_t) (WRITE_OP | REG_SLV4_REG), MAGNETO_CTRL_2 };
        uint8_t rx_magneto_read[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_read, rx_magneto_read);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_REG configuration");
        }

        //1.3 - Continuous measurement mode 2 (??)
        uint8_t tx_magneto_msg[2] = { (uint8_t) (WRITE_OP | REG_SLV4_DO), MAGNETO_MSM_MODE_2 };
        uint8_t rx_magneto_msg[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_msg, rx_magneto_msg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_DO configuration");
        }

        // 1.4 - Indicamos que ejecute la transacción
        uint8_t tx_magneto_ctrl[2] = { (uint8_t) (WRITE_OP | REG_SLV4_CTRL), MAGNETO_CONFIG_1 };
        uint8_t rx_magneto_ctrl[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_ctrl, rx_magneto_ctrl);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_ctrl configuration");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay para llevar a cabo la escritura
    } //End of scope

    // Leemos el registro de control
    {
        // 1.1
        uint8_t tx_magneto[2] = { (uint8_t) (WRITE_OP | REG_SLV4_ADDR), (READ_OP | MAGNETO_WR_ADDR )};
        uint8_t rx_magneto[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto, rx_magneto);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_ADDR configuration");
        }

        // 1.2
        uint8_t tx_magneto_read[2] = { (uint8_t) (WRITE_OP | REG_SLV4_REG), MAGNETO_CTRL_2 };
        uint8_t rx_magneto_read[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_read, rx_magneto_read);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_REG configuration");
        }

        // 1.4 - Indicamos que ejecute la transacción
        uint8_t tx_magneto_ctrl[2] = { (uint8_t) (WRITE_OP | REG_SLV4_CTRL), MAGNETO_CONFIG_1 };
        uint8_t rx_magneto_ctrl[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_ctrl, rx_magneto_ctrl);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_ctrl configuration");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay para llevar a cabo la lectura

        // Leemos el registro del WHO_AM_I del magneto
        uint8_t tx_magneto_read_1[2] = { (uint8_t) (READ_OP| REG_SLV4_DI), EMPTY_MESSAGE };
        uint8_t rx_magneto_read_1[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_read_1, rx_magneto_read_1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV0_REG configuration (2.2)");
        }
    } //End of scope

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

    return ESP_OK;
}

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