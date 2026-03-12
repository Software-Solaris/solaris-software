#include "core/core.h"
#include "icm20948.h"
#include "spi.h"

#include "gpio_int.h"
#include "driver/gpio.h"
#include "core/returntypes.h"

#include "spi.h"
#include "spp_log.h"
#include "osal/task.h"

#include "services/databank/databank.h"
#include "services/db_flow/db_flow.h"

/**
 * @file main.c
 * @brief Application entry point for ICM20948 DMP initialization and FIFO
 *        polling.
 */
#include "bmp_service.h"

static const char* TAG = "MAIN";

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
    retval_t ret;

    Core_Init();
    SPP_LOGI(TAG, "Boot");

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
    ret = SPP_DATABANK_init();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "Databank init failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    ret = DB_FLOW_Init();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "DB_FLOW init failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    ret = SPP_HAL_SPI_BusInit();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPI bus init failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    // Keep handler order: 1st ICM dummy, 2nd BMP
    void* p_spi_icm_dummy = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_spi_icm_dummy);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPI dev init ICM dummy failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    void* p_spi_bmp = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_spi_bmp);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPI dev init BMP failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    SPP_LOGI(TAG, "BMP service init");
    ret = BMP_ServiceInit(p_spi_bmp);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "BMP_ServiceInit failed ret=%d", (int)ret);
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    SPP_LOGI(TAG, "BMP service start");
    ret = BMP_ServiceStart();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "BMP_ServiceStart failed ret=%d", (int)ret);
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    SPP_LOGI(TAG, "Main idle");
    for (;;) { SPP_OSAL_TaskDelay(1000); }
}