/**
 * @file main.c
 * @brief Application entry point — boots SPP v2 and starts sensor services.
 *
 * Port registration must happen before SPP_Core_init():
 *   1. SPP_Core_setOsalPort(&g_freertosOsalPort)
 *   2. SPP_Core_setHalPort(&g_esp32HalPort)
 *   3. SPP_Core_init()   — initialises Log, Databank, DbFlow
 */

#include "spp/core/core.h"
#include "spp/core/returntypes.h"
#include "spp/hal/spi.h"
#include "spp/osal/task.h"
#include "spp/osal/port.h"
#include "spp/hal/port.h"
#include "spp/services/log.h"

#include "bmpService.h"

/* ----------------------------------------------------------------
 * External port objects (defined in spp_port_wrapper)
 * ---------------------------------------------------------------- */

extern const SPP_OsalPort_t g_freertosOsalPort;
extern const SPP_HalPort_t  g_esp32HalPort;

/* ----------------------------------------------------------------
 * Private constants
 * ---------------------------------------------------------------- */

static const char *TAG = "MAIN";

/** @brief Device index for the ICM-20948 (registered first on the SPI bus). */
#define K_MAIN_SPI_IDX_ICM (0U)

/** @brief Device index for the BMP-390. */
#define K_MAIN_SPI_IDX_BMP (1U)

/* ----------------------------------------------------------------
 * app_main
 * ---------------------------------------------------------------- */

void app_main(void)
{
    retval_t ret;

    /* --- Port registration ---------------------------------------- */
    ret = SPP_Core_setOsalPort(&g_freertosOsalPort);
    if (ret != SPP_OK)
    {
        /* Cannot log yet — OSAL not ready */
        for (;;) { /* hang */ }
    }

    ret = SPP_Core_setHalPort(&g_esp32HalPort);
    if (ret != SPP_OK)
    {
        for (;;) { /* hang */ }
    }

    /* --- Core init (Log + Databank + DbFlow) ----------------------- */
    ret = SPP_Core_init();
    if (ret != SPP_OK)
    {
        for (;;) { /* hang */ }
    }

    SPP_LOGI(TAG, "Boot");

    /* --- SPI bus init --------------------------------------------- */
    ret = SPP_Hal_spiBusInit();
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "SPI bus init failed");
        for (;;) { SPP_Osal_taskDelayMs(1000U); }
    }

    /* ICM-20948 device — must be added first (index 0) */
    void *p_spiIcm = SPP_Hal_spiGetHandle(K_MAIN_SPI_IDX_ICM);
    ret = SPP_Hal_spiDeviceInit(p_spiIcm);
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "SPI device init ICM failed");
        for (;;) { SPP_Osal_taskDelayMs(1000U); }
    }

    /* BMP-390 device — second handle (index 1) */
    void *p_spiBmp = SPP_Hal_spiGetHandle(K_MAIN_SPI_IDX_BMP);
    ret = SPP_Hal_spiDeviceInit(p_spiBmp);
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "SPI device init BMP failed");
        for (;;) { SPP_Osal_taskDelayMs(1000U); }
    }

    /* --- BMP390 service ------------------------------------------- */
    SPP_LOGI(TAG, "BMP service init");
    ret = BMP_ServiceInit(p_spiBmp);
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "BMP_ServiceInit failed ret=%d", (int)ret);
        for (;;) { SPP_Osal_taskDelayMs(1000U); }
    }

    SPP_LOGI(TAG, "BMP service start");
    ret = BMP_ServiceStart();
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "BMP_ServiceStart failed ret=%d", (int)ret);
        for (;;) { SPP_Osal_taskDelayMs(1000U); }
    }

    SPP_LOGI(TAG, "Main idle");
    for (;;)
    {
        SPP_Osal_taskDelayMs(1000U);
    }
}
