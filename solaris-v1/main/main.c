#include "core/core.h"
#include "icm20948.h"
#include "spi.h"

#include "gpio_int.h"
#include "driver/gpio.h"
#include "core/returntypes.h"

#include "spp_log.h"
#include "osal/task.h"

#include "services/databank/databank.h"
#include "services/db_flow/db_flow.h"
#include "bmp_service.h"

#include <stddef.h>

/**
 * @file main.c
 * @brief Application entry point for ICM20948 initialization, BMP service
 *        startup and FIFO polling.
 */

static const char *TAG = "MAIN";

/**
 * @brief Main application entry point.
 *
 * This function initializes the core services, configures the SPI bus,
 * initializes both SPI devices in order (ICM first, BMP second), configures
 * the ICM20948, initializes the databank and DB flow services, starts the BMP
 * service and continuously polls the ICM20948 FIFO.
 */
void app_main(void)
{
    retval_t ret = SPP_ERROR;
    void *p_spiIcm = NULL;
    void *p_spiBmp = NULL;

    Core_Init();

    SPP_LOGI(TAG, "Boot");

    ret = SPP_HAL_SPI_BusInit();
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "SPI bus init failed");
        for (;;)
        {
            SPP_OSAL_TaskDelay(1000);
        }
    }

    /* ----------------------------------------------------------------
     * ICM SPI device init
     * ---------------------------------------------------------------- */
    p_spiIcm = SPP_HAL_SPI_GetHandler();
    if (p_spiIcm == NULL)
    {
        SPP_LOGE(TAG, "SPI handler ICM is NULL");
        for (;;)
        {
            SPP_OSAL_TaskDelay(1000);
        }
    }

    ret = SPP_HAL_SPI_DeviceInit(p_spiIcm);
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "SPI dev init ICM failed");
        for (;;)
        {
            SPP_OSAL_TaskDelay(1000);
        }
    }

    ret = ICM20948_configDmpInit(p_spiIcm);
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "ICM20948_configDmpInit failed");
        for (;;)
        {
            SPP_OSAL_TaskDelay(1000);
        }
    }

    /* ----------------------------------------------------------------
     * Common services init
     * ---------------------------------------------------------------- */
    ret = SPP_DATABANK_init();
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "Databank init failed");
        for (;;)
        {
            SPP_OSAL_TaskDelay(1000);
        }
    }

    ret = DB_FLOW_Init();
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "DB_FLOW init failed");
        for (;;)
        {
            SPP_OSAL_TaskDelay(1000);
        }
    }

    /* ----------------------------------------------------------------
     * BMP SPI device init
     * ---------------------------------------------------------------- */
    p_spiBmp = SPP_HAL_SPI_GetHandler();
    if (p_spiBmp == NULL)
    {
        SPP_LOGE(TAG, "SPI handler BMP is NULL");
        for (;;)
        {
            SPP_OSAL_TaskDelay(1000);
        }
    }

    ret = SPP_HAL_SPI_DeviceInit(p_spiBmp);
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "SPI dev init BMP failed");
        for (;;)
        {
            SPP_OSAL_TaskDelay(1000);
        }
    }

    SPP_LOGI(TAG, "BMP service init");
    ret = BMP_ServiceInit(p_spiBmp);
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "BMP_ServiceInit failed ret=%d", (int)ret);
        for (;;)
        {
            SPP_OSAL_TaskDelay(1000);
        }
    }

    SPP_LOGI(TAG, "BMP service start");
    ret = BMP_ServiceStart();
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "BMP_ServiceStart failed ret=%d", (int)ret);
        for (;;)
        {
            SPP_OSAL_TaskDelay(1000);
        }
    }

    SPP_LOGI(TAG, "Main idle");

    while (true)
    {
        ICM20948_checkFifoData(p_spiIcm);
    }
}