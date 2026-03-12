#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "core/returntypes.h"
#include "core/core.h"
#include "icm20948.h"
#include "spi.h"

#include "gpio_int.h"
#include "driver/gpio.h"

#include <unistd.h>

/**
 * @file main.c
 * @brief Application entry point for ICM20948 DMP initialization and FIFO
 *        polling.
 */

/**
 * @brief Main application entry point.
 *
 * This function initializes the core services, configures the SPI bus,
 * initializes the ICM20948 device, performs the full DMP initialization
 * sequence and continuously polls the FIFO for new DMP packets.
 */
void app_main(void)
{
    retval_t ret = SPP_ERROR;
    ICM20948_Data_t s_icm20948Data;

    /* Optional startup delay. */
    /* sleep(5); */

    Core_Init();

    SPP_HAL_SPI_BusInit();

    ret = ICM20948_init((void *)&s_icm20948Data);
    if (ret != SPP_OK)
    {
        return;
    }

    ret = ICM20948_configDmpInit((void *)&s_icm20948Data);
    if (ret != SPP_OK)
    {
        return;
    }

    while (true)
    {
        ICM20948_checkFifoData((void *)&s_icm20948Data);
    }
}