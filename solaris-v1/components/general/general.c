#include "general.h"
#include "icm20948.h"
#include "bmp390.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "General";

esp_err_t init_common_sensors(data_t *icm, data_t *baro)
{
    esp_err_t ret;

    // Inicializar la ICM20948 usando el struct proporcionado.
    ret = icm20948_init(icm);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando ICM20948: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inicialización de la IMU exitosa");

    /*
    // Inicializar el barómetro (BMP390) usando el struct proporcionado.
    ret = bmp390_init(baro);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando BMP390: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inicialización del barómetro exitosa");
    */
   return ESP_OK;
}

void read_common_sensors(data_t *icm, data_t *baro) {
    esp_err_t ret = icm20948_get_measurements(icm);


    /*
    int32_t pressure, temperature;
    float altitude;
    esp_err_t ret = bmp390_get_measurements(baro, &pressure, &temperature, &altitude);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Presión: %ld Pa | Temp (raw): %ld | Altitud: %.2f m",
                 pressure, temperature, altitude);
    } else {
        ESP_LOGE(TAG, "Error al obtener mediciones del BMP390: %d", ret);
    }
    */
}
