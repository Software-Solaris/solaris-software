#include "mpu9250.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MainApp";

mpu9250_t dev; 

int app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(5000)); // espera 5000 ms (5 segundos)

    esp_err_t ret = mpu9250_init(&dev);
    if (ret != ESP_OK) {
        ESP_LOGE (TAG, "Error inicializando MPU9250: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Inicialización exitosa");

    return ESP_OK;
}

