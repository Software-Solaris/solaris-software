#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "macros.h"
#include "icm20948.h"


static const char *TAG = "MainApp";

int app_main(void)
{

    // Initial struct for setting up SPI communication
    esp_err_t ret;
    data_t icm = {0};

    //Time to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));

    //---------INIT---------
    ret = icm20948_init(&icm);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error at ICM20948 init: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "ICM20948 init succed");

    //---------CONFIG & CHECK---------
    ret = icm20948_config(&icm);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error at ICM20948 setup: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "ICM20948 setup succed");

    //---------PREPARE READ---------
    ret = icm20948_prepare_read(&icm);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error at ICM20948 calibration: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "ICM20948 calibration succed");

    //---------READ LOOP---------
    while (1) {
        ret = icm20948_read_measurements(&icm);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    return ESP_OK;
}
