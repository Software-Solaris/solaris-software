/**
 * @file general.h
 * @brief Legacy common sensor initialisation and reading helpers.
 *
 * Provides high-level functions for initialising, configuring, calibrating
 * and reading the IMU and barometer using the legacy @ref data_t SPI
 * context. These helpers are used by early Solaris versions (v0.x) and are
 * superseded by the SPP-based driver APIs in v1.
 */

#ifndef GENERAL_H
#define GENERAL_H

#include "macros.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialises SPI communication for IMU and barometer.
 *
 * @param[in,out] icm  Pointer to the IMU SPI context.
 * @param[in,out] baro Pointer to the barometer SPI context.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t init_common_sensors(data_t *icm, data_t *baro);

/**
 * @brief Applies sensor-specific configuration (wake, mode changes, etc.).
 *
 * @param[in,out] icm  Pointer to the IMU SPI context.
 * @param[in,out] baro Pointer to the barometer SPI context.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t configure_common_sensors(data_t *icm, data_t *baro);

/**
 * @brief Runs the calibration sequence for both sensors.
 *
 * @param[in,out] icm  Pointer to the IMU SPI context.
 * @param[in,out] baro Pointer to the barometer SPI context.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t calibrate_common_sensors(data_t *icm, data_t *baro);

/**
 * @brief Reads and logs measurements from both sensors.
 *
 * @param[in,out] icm  Pointer to the IMU SPI context.
 * @param[in,out] baro Pointer to the barometer SPI context.
 */
void read_common_sensors(data_t *icm, data_t *baro);

#ifdef __cplusplus
}
#endif

#endif /* GENERAL_H */
