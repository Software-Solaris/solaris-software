#include "gpio_int.h"
#include "esp_log.h"

static const char *TAG = "GPIO_INT";
// static bool s_int_flag = false;
int data_isr = 0;

//Configuración del pin INT
esp_err_t int_gpio_init(void)
{
    esp_err_t ret = ESP_OK;

    //Limpiar Config
    ret = gpio_reset_pin(INT_GPIO);

    if (ret != ESP_OK){
        ESP_LOGE (TAG, "Error reseteando el INT");
        return ret;
    }

    //Config
    const gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << INT_GPIO),        // GPIO5
        .mode          = GPIO_MODE_INPUT,           // Entrada
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_POSEDGE          //Flanco ascendente
    };//Hai que ver como funciona o INT da IMU para saber que habilitar e que tipo de flanco buscar

    ret = gpio_config(&io_conf);

    if (ret != ESP_OK){
        ESP_LOGE (TAG, "Error configurando el INT");
        return ret;
    }

    return ret;
}

//ISR handler
void isr_handler(void *data_isr)
{
    int *flag = (int*) data_isr;
    static int i = 1;
    *flag = i;  // o lo que quieras marcar
    i++;
}

//ISR (desde o IDF, sin RTOS)
esp_err_t isr_config(void)
{
    esp_err_t ret = ESP_OK;

    
    //Install the GPIO driver's ETS_GPIO_INTR_SOURCE ISR handler service, which allows per-pin GPIO interrupt handlers.
    ret = gpio_install_isr_service(FLAG);
    
    if (ret == ESP_ERR_INVALID_STATE) ret = ESP_OK;   // pode dar "error" se xa estaba instalada, tratamolo como ok
    if (ret != ESP_OK) return ret;
    
    //Add ISR handler for the corresponding GPIO pin.
    ret = gpio_isr_handler_add(INT_GPIO, isr_handler, (void*)&data_isr);

    return ret;
}