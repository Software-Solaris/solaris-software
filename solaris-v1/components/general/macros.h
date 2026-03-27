/**
 * @file macros.h
 * @brief Legacy SPI pin definitions and common data structure.
 *
 * Defines the SPI bus pins used by the ESP32-S3 board and provides the
 * legacy @ref data_t structure that bundles an SPI device handle with its
 * bus/device/transaction configuration. Used by early Solaris versions
 * (v0.x); superseded by the SPP HAL SPI API in v1.
 */

#ifndef MACROS_H
#define MACROS_H

#include <driver/spi_master.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief SPI host peripheral used for sensor communication. */
#define SPI_HOST_USED SPI2_HOST

/** @brief GPIO pin for SPI CIPO (Controller In, Peripheral Out). */
#define PIN_NUM_CIPO       47

/** @brief GPIO pin for SPI COPI (Controller Out, Peripheral In). */
#define PIN_NUM_COPI       38

/** @brief GPIO pin for SPI clock. */
#define PIN_NUM_CLK        48

/**
 * @brief Legacy SPI device context.
 *
 * Bundles the ESP-IDF SPI handle together with bus, device and transaction
 * descriptors plus a small register-level I/O buffer.
 */
typedef struct {
    spi_device_handle_t handle;                /**< ESP-IDF SPI device handle.        */
    spi_bus_config_t buscfg;                   /**< SPI bus configuration.            */
    spi_device_interface_config_t devcfg;      /**< SPI device configuration.         */
    spi_transaction_t trans_desc;               /**< Reusable transaction descriptor.  */
    uint8_t sensor_id;                          /**< Sensor identifier byte.           */
    uint8_t reg;                                /**< Target register address.          */
    uint8_t data;                               /**< Single-byte data buffer.          */
} data_t;

#ifdef __cplusplus
}
#endif

#endif /* MACROS_H */
