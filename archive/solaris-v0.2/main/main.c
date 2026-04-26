#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "macros.h"
#include "icm20948.h"
#include "kalman.h"
#include "esp_timer.h"


static const char *TAG = "MainApp";
static const char *k_tag = "Kalman";

int app_main(void)
{
    // Initial struct for setting up SPI communication
    esp_err_t ret;
    data_t icm = {0};

    //Time to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));

    //---------INIT---------
    ret = icm20948_init(&icm);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error at ICM20948 init: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "ICM20948 init succed");

    //---------CONFIG & CHECK---------
    ret = icm20948_config(&icm);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error at ICM20948 setup: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "ICM20948 setup succed");

    //---------PREPARE READ---------
    ret = icm20948_prepare_read(&icm);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error at ICM20948 calibration: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "ICM20948 calibration succed");


    // Kalman filter
    sensor_data data = {0};
    kalman_state kal;

    float Q_init[16] = {0};
    float R_init[3] = {0.001f, 0.001f, 0.001f};

    SPP_SERVICES_KALMAN_ekfInit(&kal, &data, 0.01f, Q_init, R_init);

    int64_t last_time_us = esp_timer_get_time();

    ESP_LOGI(k_tag, "Services ready — entering superloop");

    /* ----------------------------------------------------------------
     * Superloop
     * ---------------------------------------------------------------- */

    float accx = 0.0f;
    float accy = 0.0f;
    float accz = 0.0f;
    float gyrox = 0.0f;
    float gyroy = 0.0f;
    float gyroz = 0.0f;
    float tot_offset_ax = 0.0f;
    float tot_offset_ay = 0.0f;
    float tot_offset_az = 0.0f;
    float tot_offset_gx = 0.0f;
    float tot_offset_gy = 0.0f;
    float tot_offset_gz = 0.0f;

    for (int j = 0; j < 100; j++)

    {
        float ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
        KALMAN_offsets(&icm, &accx, &accy, &accz, &gyrox, &gyroy, &gyroz, &ax_offset, &ay_offset,
                       &az_offset, &gx_offset, &gy_offset, &gz_offset);
        tot_offset_ax += ax_offset;
        tot_offset_ay += ay_offset;
        tot_offset_az += az_offset;
        tot_offset_gx += gx_offset;
        tot_offset_gy += gy_offset;
        tot_offset_gz += gz_offset;
    }

    float ax_offset = tot_offset_ax / 100.0f;
    float ay_offset = tot_offset_ay / 100.0f;
    float az_offset = tot_offset_az / 100.0f;
    float gx_offset = tot_offset_gx / 100.0f;
    float gy_offset = tot_offset_gy / 100.0f;
    float gz_offset = tot_offset_gz / 100.0f;

    ESP_LOGI("OFFSETS", "ax: %.2f, ay: %.2f, az: %.2f | gx: %.2f, gy: %.2f, gz: %.2f", ax_offset,
             ay_offset, az_offset, gx_offset, gy_offset, gz_offset);
    for (;;)
    {
        ret = KALMAN_readFunction(&icm, &accx, &accy, &accz, &gyrox, &gyroy, &gyroz, ax_offset,
                                  ay_offset, az_offset, gx_offset, gy_offset, gz_offset);
        if (ret != ESP_OK)
        {
            ESP_LOGE(k_tag, "Failed to get ICM20948 measurements ret=%d", (int)ret);
            continue;
        }

        data.acc_data[0] = accx;
        data.acc_data[1] = accy;
        data.acc_data[2] = accz;

        data.gyro_data[0] = gyrox * DEG_TO_RAD;
        data.gyro_data[1] = gyroy * DEG_TO_RAD;
        data.gyro_data[2] = gyroz * DEG_TO_RAD;

        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_time_us) / 1000000.0f;
        last_time_us = now_us;

        SPP_SERVICES_KALMAN_run(&kal, &data, dt);

        ESP_LOGI("EKF", "QUAT, (w=%.4f, x=%.4f, y=%.4f, z=%.4f)", kal.qw, kal.qx, kal.qy, kal.qz);
    }

    return ESP_OK;
}
