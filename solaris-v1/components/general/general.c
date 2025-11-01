#include "general.h"
#include "icm20948.h"
#include "bmp390.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "General";
static esp_err_t ret;

esp_err_t init_common_sensors(data_t *icm, data_t *baro) {

    // Inicializar la (ICM20948) usando el struct proporcionado.
    ret = icm20948_init(icm);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error at ICM20948 init: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "ICM20948 init succed");

    // // Inicializar el barómetro (BMP390) usando el struct proporcionado.
    // ret = bmp390_init(baro);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Error inicializando BMP390: %d", ret);
    //     return ret;
    // }
    // ESP_LOGI(TAG, "Inicialización del barómetro exitosa");

   return ESP_OK;
}

esp_err_t configure_common_sensors(data_t *icm, data_t *baro) {

    // Configuración del ICM20948
    ret = icm20948_config(icm);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error at ICM20948 setup: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "ICM20948 setup succed");

    // // Configuración del BMP390
    // ret = bmp390_config(baro);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Error at BMP390 setup: %d", ret);
    //     return ret;
    // }
    // ESP_LOGI(TAG, "BMP390 setup succed");

    return ESP_OK;
}

esp_err_t calibrate_common_sensors(data_t *icm, data_t *baro) {
    // Calibrado del ICM20948
    ret = icm20948_prepare_read(icm);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error at ICM20948 calibration: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "ICM20948 calibration succed");

    // // Calibrado del BMP390
    // ret = bmp390_prepare_read(baro);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Error at BMP390 calibration: %d", ret);
    //     return ret;
    // }
    // ESP_LOGI(TAG, "BMP390 calibration succed");

   return ESP_OK;
}

void read_common_sensors(data_t *icm, data_t *baro) {

    ret = icm20948_read_measurements(icm);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Some ICM20948 measurement failed: %d", ret);
    }

    // ret = bmp390_read_measurements(baro);

    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Some BMP390 measurement failed: %d", ret);
    // }
}
