#ifndef BMP_SERVICE_H
#define BMP_SERVICE_H

#include "core/returntypes.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
 * @file bmp_service.h
 * @brief Public API for the BMP390 service layer.
 *
 * This module provides a service abstraction on top of the BMP390 driver.
 * It handles initialization, task creation, interrupt management and
 * periodic acquisition of pressure and temperature data from the sensor.
 *
 * The service typically:
 * - Initializes the BMP390 driver
 * - Configures the sensor
 * - Manages the data-ready interrupt
 * - Runs a task responsible for reading sensor data and publishing it
 *   through the system data services.
 */

    /**
 * @brief Initializes the BMP390 service.
 *
 * This function prepares the BMP390 service context and associates it
 * with the SPI device handler used to communicate with the sensor.
 * It must be called before starting the service.
 *
 * @param[in] p_spi_bmp Pointer to the SPI device handler assigned to the BMP390.
 *
 * @return retval_t
 * - SPP_OK if initialization succeeds
 * - Error code otherwise
 */
    retval_t BMP_ServiceInit(void *p_spi_bmp);

    /**
 * @brief Starts the BMP390 service.
 *
 * This function starts the internal service execution, typically by
 * creating and launching the BMP390 acquisition task and enabling
 * the sensor interrupt mechanism.
 *
 * @return retval_t
 * - SPP_OK if the service starts correctly
 * - Error code otherwise
 */
    retval_t BMP_ServiceStart(void);

#ifdef __cplusplus
}
#endif

#endif /* BMP_SERVICE_H */