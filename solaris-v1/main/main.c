/**
 * @file main.c
 * @brief Application entry point — baremetal cooperative scheduler.
 *
 * Uses the baremetal OSAL port (cooperative round-robin, no FreeRTOS tasks)
 * and the polling SPI HAL (spi_device_polling_transmit, no FreeRTOS semaphore).
 *
 * Boot sequence:
 *   1. Register baremetal OSAL + ESP32 polling HAL ports
 *   2. SPP_Core_init()  — Log, Databank, DbFlow
 *   3. SPI bus + device init
 *   4. Register and init/start services
 *   5. SPP_Baremetal_run() loop — cooperative scheduler tick
 */

#include "spp/core/core.h"
#include "spp/core/returntypes.h"
#include "spp/hal/spi.h"
#include "spp/osal/port.h"
#include "spp/hal/port.h"
#include "spp/services/log/log.h"
#include "spp/services/service.h"
#include "spp/services/bmp390/bmp390.h"

#include "macros_esp32.h"

/* ----------------------------------------------------------------
 * External port objects (defined in spp_port_wrapper)
 * ---------------------------------------------------------------- */

extern const SPP_OsalPort_t g_baremetalOsalPort;
extern const SPP_HalPort_t  g_esp32BaremetalHalPort;

/**
 * @brief Run one cooperative scheduler tick.
 *
 * Declared here to avoid a separate header dependency.
 * Defined in ports/baremetal/osal/osal_baremetal.c.
 */
void SPP_Baremetal_run(void);

/* ----------------------------------------------------------------
 * Private constants
 * ---------------------------------------------------------------- */

static const char *TAG = "MAIN";

/** @brief GPIO pin for the BMP390 data-ready interrupt. */
#define K_MAIN_BMP390_INT_PIN (5U)

/* ----------------------------------------------------------------
 * Service instances (static allocation)
 * ---------------------------------------------------------------- */

static BMP390_ServiceCtx_t s_bmpCtx;

static const BMP390_ServiceCfg_t s_bmpCfg =
{
    .spiDevIdx    = K_ESP32_SPI_IDX_BMP,
    .intPin       = K_MAIN_BMP390_INT_PIN,
    .intIntrType  = 1U,
    .intPull      = 0U,
    .sdCfg =
    {
        .p_basePath          = "/sdcard",
        .spiHostId           = K_ESP32_SPI_HOST,
        .pinCs               = K_ESP32_PIN_CS_SDC,
        .maxFiles            = 5U,
        .allocationUnitSize  = (16U * 1024U),
        .formatIfMountFailed = false,
    },
    .p_logFilePath = "/sdcard/log.txt",
    .logMaxPackets = 10U,
};

/* ----------------------------------------------------------------
 * app_main
 * ---------------------------------------------------------------- */

void app_main(void)
{
    SPP_RetVal_t ret;

    /* --- Port registration ---------------------------------------- */
    ret = SPP_Core_setOsalPort(&g_baremetalOsalPort);
    if (ret != K_SPP_OK) { for (;;) {} }

    ret = SPP_Core_setHalPort(&g_esp32BaremetalHalPort);
    if (ret != K_SPP_OK) { for (;;) {} }

    /* --- Core init ------------------------------------------------- */
    ret = SPP_Core_init();
    if (ret != K_SPP_OK) { for (;;) {} }

    SPP_LOGI(TAG, "Boot (baremetal)");

    /* --- SPI bus + devices ----------------------------------------- */
    ret = SPP_HAL_spiBusInit();
    if (ret != K_SPP_OK)
    {
        SPP_LOGE(TAG, "SPI bus init failed");
        for (;;) {}
    }

    void *p_spiIcm = SPP_HAL_spiGetHandle(K_ESP32_SPI_IDX_ICM);
    ret = SPP_HAL_spiDeviceInit(p_spiIcm);
    if (ret != K_SPP_OK)
    {
        SPP_LOGE(TAG, "SPI ICM init failed");
        for (;;) {}
    }

    void *p_spiBmp = SPP_HAL_spiGetHandle(K_ESP32_SPI_IDX_BMP);
    ret = SPP_HAL_spiDeviceInit(p_spiBmp);
    if (ret != K_SPP_OK)
    {
        SPP_LOGE(TAG, "SPI BMP init failed");
        for (;;) {}
    }

    (void)p_spiIcm;
    (void)p_spiBmp;

    /* --- Service registry ------------------------------------------ */
    ret = SPP_Service_register(&g_bmp390ServiceDesc, &s_bmpCtx, &s_bmpCfg);
    if (ret != K_SPP_OK)
    {
        SPP_LOGE(TAG, "Service register failed ret=%d", (int)ret);
        for (;;) {}
    }

    ret = SPP_Service_initAll();
    if (ret != K_SPP_OK)
    {
        SPP_LOGE(TAG, "Service initAll failed ret=%d", (int)ret);
        for (;;) {}
    }

    ret = SPP_Service_startAll();
    if (ret != K_SPP_OK)
    {
        SPP_LOGE(TAG, "Service startAll failed ret=%d", (int)ret);
        for (;;) {}
    }

    SPP_LOGI(TAG, "Entering baremetal loop");

    /* --- Cooperative scheduler loop -------------------------------- */
    for (;;)
    {
        SPP_Baremetal_run();
    }
}
