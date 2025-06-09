#include "general.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "macros.h"

static const char *TAG = "MainApp";

int app_main(void)
{
    // Declarar e inicializar los structs de los sensores (inicializados a cero)
    data_t icm_dev = {0};
    data_t baro_dev = {0};

    // Esperar 5 segundos antes de iniciar (para estabilizar el sistema)
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Inicializar los sensores usando los structs ya creados.
    if (init_common_sensors(&icm_dev, &baro_dev) != ESP_OK) {
        ESP_LOGE(TAG, "Error en la inicialización de los sensores.");
        return ESP_FAIL;
    }

    // Bucle principal: leer los sensores cada 2 segundos.
    while (1) {
        read_common_sensors(&icm_dev, &baro_dev);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    return ESP_OK;
}
