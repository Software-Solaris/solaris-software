#include "general.h"
#include "icm20948.h"
#include "bmp390.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "General";

esp_err_t init_common_sensors(data_t *icm, data_t *baro) {
    esp_err_t ret;

    // Inicializar la ICM20948 usando el struct proporcionado.
    ret = icm20948_init(icm);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error at ICM20948 init: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "ICM20948 init succed");

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

esp_err_t configure_common_sensors(data_t *icm, data_t *baro) {
    esp_err_t ret;

    // Configuración del ICM20948
    ret = icm20948_config(icm);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error at ICM20948 setup: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "ICM20948 setup succed");

    // IMPORTANTE!!: Aquí va la función de configuración del barómetro

    return ESP_OK;
}

void read_common_sensors(data_t *icm, data_t *baro) {
    esp_err_t ret;

    ret = icm20948_get_measurements(icm);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Some ICM20948 measurement failed: %d", ret);
    }


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
