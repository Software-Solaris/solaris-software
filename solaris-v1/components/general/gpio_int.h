#include "driver/gpio.h"
#include "esp_err.h"

#define INT_GPIO GPIO_NUM_17
#define FLAG ESP_INTR_FLAG_IRAM //caso crítico, hai mais niveles de prioridades; ESP_INTR_FLAG_EDGE para flancos pode estar ben


esp_err_t int_gpio_init(void);
esp_err_t isr_config(void);
void isr_handler(void* arg); //IRAM_ATTR solo se se usa a flag de caso crítico (caché)git stash