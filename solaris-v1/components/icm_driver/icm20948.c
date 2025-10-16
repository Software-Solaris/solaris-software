#include "icm20948.h"
#include "macros.h"
#include "esp_log.h"
#include "driver/spi_common.h"

static const char* TAG = "ICM20948"; 
static esp_err_t ret;

//-----------------------------AUX-----------------------------
esp_err_t send_message(data_t *p_dev, uint8_t tx[2], uint8_t rx[2]) {
    // Transaction parameters set up
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

//-----------------------------INIT-------------------------------
esp_err_t icm20948_init(data_t *p_dev) {

    // 1. Init of SPI communication
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

    // 2. Configures SPI (CS, speed, mode, ...)
    p_dev->devcfg.clock_speed_hz = 100000;
    p_dev->devcfg.mode = 3;           // Try on mode 0 if it fails
    p_dev->devcfg.spics_io_num = PIN_NUM_CS;
    p_dev->devcfg.queue_size = 20; // To execute queue SPI transaction
    p_dev->devcfg.address_bits = 0;
    p_dev->devcfg.command_bits = 0;
    p_dev->devcfg.dummy_bits = 0;
    p_dev->devcfg.flags = 0;
    p_dev->devcfg.duty_cycle_pos = 128;
    p_dev->devcfg.pre_cb = NULL;
    p_dev->devcfg.post_cb = NULL;
    vTaskDelay(pdMS_TO_TICKS(100)); // Aditional waiting of 100 ms

    ret = spi_bus_add_device(SPI_HOST_USED, &p_dev->devcfg, &p_dev->handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error spi_bus_add_device on ICM20948: %d", ret);
        return ret;
    } 
    ESP_LOGI(TAG, "ICM20948 added to SPI bus.");

    return ESP_OK;
}


esp_err_t icm20948_config(data_t *p_dev) {

    // Reset of ICM: writing 0x80 on PWR_MGMT_1
    uint8_t tx_reset[2] = { (uint8_t) (WRITE_OP | REG_PWR_MGMT_1), BIT_H_RESET };
    uint8_t rx_reset[2] = { 0, 0 };

    ret = send_message(p_dev, tx_reset, rx_reset);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor reset returned an error: %d", ret);
        return ret;
    }

    // Wake up the ICM from "SLEEP_MODE" and deactivation of temperature sensor:  writing 0x09 on PWR_MGMT_1
    uint8_t tx_sleep_off[2] = { (uint8_t) (WRITE_OP | REG_PWR_MGMT_1), 0x09 };
    uint8_t rx_sleep_off[2] = { 0, 0 };
    vTaskDelay(pdMS_TO_TICKS(20));
    ret = send_message(p_dev, tx_sleep_off, rx_sleep_off);


    // Reading of WHO_AM_I content: reading 0x00
    uint8_t tx_who_am_i[2] = { (uint8_t) (READ_OP | REG_WHO_AM_I), EMPTY_MESSAGE };
    uint8_t rx_who_am_i[2] = { 0, 0 };

    ret = send_message(p_dev, tx_who_am_i, rx_who_am_i);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I reading failed: %d", ret);
        return ret;
    } else {
        p_dev->sensor_id = rx_who_am_i[1]; // Data is stored on the second element
        ESP_LOGI(TAG, "WHO_AM_I register has 0x%02X | should be: 0xEA", p_dev->sensor_id);
    }

    // Enable ICM resources: activate FIFO, internal I2C and DMP in USER_CTRL register
    uint8_t tx_user_ctrl[2] = { (uint8_t) (WRITE_OP | REG_USER_CTRL), USER_CTRL_CONFIG };
    uint8_t rx_user_ctrl[2] = { 0, 0 };

    ret = send_message(p_dev, tx_user_ctrl, rx_user_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error on USER_CTRL sub-modules activation");
    }

    ret = send_message(p_dev, tx_user_ctrl, rx_user_ctrl);


    // Swap to bank 3
    uint8_t tx_bank_sel[2] = { (uint8_t) (WRITE_OP | REG_BANK_SEL), 0x30 };
    uint8_t rx_bank_sel[2] = { 0, 0 };

    ret = send_message(p_dev, tx_bank_sel, rx_bank_sel);

    // I2C_MST_ODR_CONFIG is set on 0x00 for 1.1kHz sample rate
    // Definition of internal I2C transaction speed: writing 0x07 on I2C_MST_CTRL
    uint8_t tx_i2c_ctrl[2] = { (uint8_t) (WRITE_OP | REG_I2C_CTRL), I2C_SP_CONFIG };
    uint8_t rx_i2c_ctrl[2] = { 0, 0 };

    ret = send_message(p_dev, tx_i2c_ctrl, rx_i2c_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error on i2c transmition configuration");
    }

    /* MAGNETOMETER CONFIGURATION

        1. Use SLV_4 to get the magnetometer out of the power-off mode
            1.1 Access the magnetometer with its physical address in write mode: write 0x0C to I2C_SLV4_ADDR
            1.2. Set the “Control 2” register address: write 0x31 to I2C_SLV4_REG
            1.3. Write the message to be sent: write 0x08 (mode 2) to I2C_SLV4_DO
            1.4. Configure the transaction data: write 0x81 to I2C_SLV4_CTRL

        2. Configure periodic readings of data from SLV_0
            2.1. Access the magnetometer with its physical address in read mode: write 0x8C to I2C_SLV0_ADDR
            2.2. Set the starting address of the registers: write 0x11 to I2C_SLV0_REG
            2.3. Configure the transaction data: write 0x81 to I2C_SLV0_CTRL
    */

    // Device ID reading (using a C scope with {})
    { // First, we read magnetometer's WHO_AM_I
        uint8_t tx_magneto[2] = { (uint8_t) (WRITE_OP | REG_SLV4_ADDR), MAGNETO_RD_ADDR};
        uint8_t rx_magneto[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto, rx_magneto);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_ADDR configuration");
        }

        uint8_t tx_magneto_read[2] = { (uint8_t) (WRITE_OP | REG_SLV4_REG), MAGNETO_WHO_AM_I };
        uint8_t rx_magneto_read[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_read, rx_magneto_read);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_REG configuration");
        }

        uint8_t tx_magneto_ctrl[2] = { (uint8_t) (WRITE_OP | REG_SLV4_CTRL), MAGNETO_CONFIG_1 };
        uint8_t rx_magneto_ctrl[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_ctrl, rx_magneto_ctrl);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_ctrl configuration");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for writing

        // Reading of Magnetometer WHO_AM_I
        uint8_t tx_magneto_read_1[2] = { (uint8_t) (READ_OP| REG_SLV4_DI), EMPTY_MESSAGE };
        uint8_t rx_magneto_read_1[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_read_1, rx_magneto_read_1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV0_REG configuration (2.2)");
        } else {
            ESP_LOGI(TAG, "WHO_AM_I register has 0x%02X | should be: 0x09", rx_magneto_read_1);
        }
    } //End of scope

    // Activation of the magnetometer - measurement mode 2
    {
        // 1.1
        uint8_t tx_magneto[2] = { (uint8_t) (WRITE_OP | REG_SLV4_ADDR), MAGNETO_WR_ADDR};
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

        // 1.3 - Continuous measurement mode 2
        uint8_t tx_magneto_msg[2] = { (uint8_t) (WRITE_OP | REG_SLV4_DO), MAGNETO_MSM_MODE_2 };
        uint8_t rx_magneto_msg[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_msg, rx_magneto_msg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_DO configuration");
        }

        // 1.4
        uint8_t tx_magneto_ctrl[2] = { (uint8_t) (WRITE_OP | REG_SLV4_CTRL), MAGNETO_CONFIG_1 };
        uint8_t rx_magneto_ctrl[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_ctrl, rx_magneto_ctrl);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV4_ctrl configuration");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    } //End of scope

    {
        // 2.1 
        uint8_t tx_magneto_1[2] = { (uint8_t) (WRITE_OP | REG_SLV0_ADDR), MAGNETO_RD_ADDR };
        uint8_t rx_magneto_1[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_1, rx_magneto_1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV0_ADDR configuration (2.1)");
        }


        // 2.2
        uint8_t tx_magneto_read_1[2] = { (uint8_t) (WRITE_OP | REG_SLV0_REG), MAGNETO_START_RD };
        uint8_t rx_magneto_read_1[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_read_1, rx_magneto_read_1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV0_REG configuration (2.2)");
        }

        // 2.3
        uint8_t tx_magneto_ctrl_1[2] = { (uint8_t) (WRITE_OP | REG_SLV0_CTRL), MAGNETO_CONFIG_2 };
        uint8_t rx_magneto_ctrl_1[2] = { 0, 0 };

        ret = send_message(p_dev, tx_magneto_ctrl_1, rx_magneto_ctrl_1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error on SLV0_CTRL configuration (2.3)");
        }

        // Swap to bank 0
        uint8_t tx_bank_sel_2[2] = { (uint8_t) (WRITE_OP | REG_BANK_SEL), 0x00 };
        uint8_t rx_bank_sel_2[2] = { 0, 0 };

        ret = send_message(p_dev, tx_bank_sel_2, rx_bank_sel_2);
    }

    return ESP_OK;
}

esp_err_t icm20948_prepare_read(data_t *p_dev) {

    // Swap to bank 2
    uint8_t tx_bank_sel_3[2] = { (uint8_t) (WRITE_OP | REG_BANK_SEL), 0x20 };
    uint8_t rx_bank_sel_3[2] = { 0, 0 };

    ret = send_message(p_dev, tx_bank_sel_3, rx_bank_sel_3);

    // Accelerometer low-pass filter configutarion: writing the configuration on ACCEL_CONFIG
    uint8_t tx_accel_conf[2] = { (uint8_t) (WRITE_OP | REG_ACCEL_CONFIG), ACCEL_FILTER_SELEC };
    uint8_t rx_accel_conf[2] = { 0, 0 };

    ret = send_message(p_dev, tx_accel_conf, rx_accel_conf);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "ACCEL_CONFIG modified succesfully, now low pass filter is activated");
    }

    // Gyroscope low-pass filter configutarion: writing the configuration on GYRO_CONFIG
    uint8_t tx_gyro_conf[2] = { (uint8_t) (WRITE_OP | REG_GYRO_CONFIG), GYRO_FILTER_SELEC };
    uint8_t rx_gyro_conf[2] = { 0, 0 };

    ret = send_message(p_dev, tx_gyro_conf, rx_gyro_conf);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "GYRO_CONFIG modified succesfully, now low pass filter is activated");
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    // Devolver a banco 0 para las lecturas de datos
    uint8_t tx_bank_sel_4[2] = { (uint8_t) (WRITE_OP | REG_BANK_SEL), 0x00 };
    uint8_t rx_bank_sel_4[2] = { 0, 0 };

    ret = send_message(p_dev, tx_bank_sel_4, rx_bank_sel_4);

    return ESP_OK;

}

esp_err_t icm20948_read_measurements(data_t *p_dev) {
    // Required variables
    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    float accel_x, accel_y, accel_z;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
    float gyro_x, gyro_y, gyro_z;
    int16_t magneto_x_raw, magneto_y_raw, magneto_z_raw;
    float magneto_x, magneto_y, magneto_z;

    // Compensation variables (these are unique for each esp32, so they must be set with the average destiation of your own esp32)
    float ax_offset = 1.622;
    float ay_offset = 0.288;
    float az_offset = -8.682;
    float gx_offset = -0.262;
    float gy_offset = -2.372;
    float gz_offset = 0.014;

    // Accel: X
    accel_x_raw = get_raw_axis_data(p_dev, REG_ACCEL_X_H, REG_ACCEL_X_L);
    accel_x = (((float)accel_x_raw / 16384.0f) * 9.80665f) + ax_offset; // Digital value -> g -> m/s2

    // Accel: Y
    accel_y_raw = get_raw_axis_data(p_dev, REG_ACCEL_Y_H, REG_ACCEL_Y_L);
    accel_y = (((float)accel_y_raw / 16384.0f) * 9.80665f) + ay_offset;

    // Accel: Z
    accel_z_raw = get_raw_axis_data(p_dev, REG_ACCEL_Z_H, REG_ACCEL_Z_L);
    accel_z = (((float)accel_z_raw / 16384.0f) * 9.80665f) + az_offset;

    // Gyro: X
    gyro_x_raw = get_raw_axis_data(p_dev, REG_GYRO_X_H, REG_GYRO_X_L);
    gyro_x = ((float)gyro_x_raw / 131.0f) + gx_offset; // Digital value -> dps (degrees per second)

    // Gyro: Y
    gyro_y_raw = get_raw_axis_data(p_dev, REG_GYRO_Y_H, REG_GYRO_Y_L);
    gyro_y = ((float)gyro_y_raw / 131.0f) + gy_offset;

    // Gyro: Z
    gyro_z_raw = get_raw_axis_data(p_dev, REG_GYRO_Z_H, REG_GYRO_Z_L);
    gyro_z = ((float)gyro_z_raw / 131.0f) + gz_offset;

    // Magneto: X
    magneto_x_raw = get_raw_axis_data(p_dev, REG_MAGNETO_X_H, REG_MAGNETO_X_L);
    magneto_x = ((float)magneto_x_raw * 0.15); // Digital value -> μT

    // Magneto: Y
    magneto_y_raw = get_raw_axis_data(p_dev, REG_MAGNETO_Y_H, REG_MAGNETO_Y_L);
    magneto_y = ((float)magneto_y_raw * 0.15);

    // Magneto: Z
    magneto_z_raw = get_raw_axis_data(p_dev, REG_MAGNETO_Z_H, REG_MAGNETO_Z_L);
    magneto_z = ((float)magneto_z_raw * 0.15);

    // Logs
    ESP_LOGI(TAG, "Accel (g)     - X: %.2f, Y: %.2f, Z: %.2f", accel_x, accel_y, accel_z);
    ESP_LOGI(TAG, "Gyro (dps     - X: %.2f, Y: %.2f, Z: %.2f", gyro_x, gyro_y, gyro_z);
    ESP_LOGI(TAG, "Magneto (μT)  - X: %.2f, Y: %.2f, Z: %.2f", magneto_x, magneto_y, magneto_z);


    return ESP_OK;
}