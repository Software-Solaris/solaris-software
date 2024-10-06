#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_PIN GPIO_NUM_2  // Change this to your LED pin number

void blink_task(void *pvParameter) {
    // Configure the GPIO pin
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_PIN, 1);  // Turn the LED on
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
        gpio_set_level(LED_PIN, 0);  // Turn the LED off
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
    }
}

void app_main(void) {
    // Create the blink task
    xTaskCreate(blink_task, "blink_task", 1024, NULL, 5, NULL);
}
//comentario