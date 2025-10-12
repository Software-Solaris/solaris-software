#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "general.h"
#include "macros.h"
#include "gpio_int.h"
// #include "core.h"


// retval_t ret = SPP_OK;
static const char *TAG = "MainApp";

int app_main(void)
{

    // ret = Core_Init();
    // Declarar e inicializar los structs de los sensores (inicializados a cero)
    data_t icm_dev = {0};
    data_t baro_dev = {0};

    // Esperar 5 segundos antes de iniciar (para estabilizar el sistema)
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Inicializar los sensores usando los structs ya creados.
    esp_err_t com_result = init_common_sensors(&icm_dev, &baro_dev);
    if (com_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed on establishing communications");
        // return ESP_FAIL;
    }

    // Configuración interna de los sensores
    esp_err_t set_up_result = configure_common_sensors(&icm_dev, &baro_dev);
    if (set_up_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed on sensors set up");
        // return ESP_FAIL;
    }

    // Calibración de los sensores para recogida de datos
    esp_err_t calib_result = calibrate_common_sensors(&icm_dev, &baro_dev);
    if (calib_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed on sensors calibration");
        // return ESP_FAIL;
    }

    // int_gpio_init();
    // isr_config();


    // Lectura periódica de los datos en los sensores
    while (1) {
        read_common_sensors(&icm_dev, &baro_dev);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    return ESP_OK;
}
