/**
 * @file gpio_int.h
 * @brief Legacy GPIO interrupt helpers for the BMP390 data-ready pin.
 *
 * Configures a single GPIO pin as an interrupt input and registers an ISR
 * handler. Used by early Solaris versions (v0.x); superseded by the SPP
 * HAL GPIO API in v1.
 */

#ifndef GPIO_INT_H
#define GPIO_INT_H

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief GPIO pin number used for the sensor interrupt. */
#define INT_GPIO 5

/**
 * @brief ISR allocation flag.
 *
 * ESP_INTR_FLAG_IRAM places the handler in IRAM for lowest latency.
 */
#define FLAG ESP_INTR_FLAG_IRAM

/**
 * @brief Configures the interrupt GPIO pin as input.
 *
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t int_gpio_init(void);

/**
 * @brief Installs the GPIO ISR service and registers the handler.
 *
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t isr_config(void);

/**
 * @brief GPIO ISR handler callback.
 *
 * Must be placed in IRAM when using ESP_INTR_FLAG_IRAM.
 *
 * @param[in] arg User-defined argument passed during ISR registration.
 */
void isr_handler(void* arg);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_INT_H */
