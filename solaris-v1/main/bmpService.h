/**
 * @file bmpService.h
 * @brief Public API for the BMP390 service layer.
 *
 * This module provides a service abstraction on top of the BMP390 driver.
 * It handles initialisation, task creation, interrupt management and
 * periodic acquisition of pressure and temperature data from the sensor.
 */

#ifndef BMP_SERVICE_H
#define BMP_SERVICE_H

#include "spp/core/returntypes.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initialise the BMP390 service.
     *
     * Associates the service with the SPI device handle for the BMP390.
     * Must be called before @ref BMP_ServiceStart().
     *
     * @param[in] p_spiBmp  SPI device handle for the BMP390 sensor.
     *
     * @return SPP_OK on success, error code otherwise.
     */
    retval_t BMP_ServiceInit(void *p_spiBmp);

    /**
     * @brief Start the BMP390 acquisition task.
     *
     * Creates the FreeRTOS task and enables the sensor data-ready interrupt.
     *
     * @return SPP_OK on success, error code otherwise.
     */
    retval_t BMP_ServiceStart(void);

#ifdef __cplusplus
}
#endif

#endif /* BMP_SERVICE_H */
