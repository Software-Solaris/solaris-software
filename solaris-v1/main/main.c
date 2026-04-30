/**
 * @file main.c
 * @brief Solaris v1 application entry point — bare-metal superloop.
 *
 * Data flow:
 *   ISR sets drdyFlag
 *     → callProducers() calls each sensor's produce()
 *       → produce() reads sensor, builds packet, calls publish()
 *         → SYNC subscribers run immediately inside publish()
 *         → other subscribers queued, dispatched one-per-call by callConsumers()
 */

#include "spp/spp.h"
#include "spp/services/bmp390/bmp390.h"
#include "spp/services/icm20948/icm20948.h"
#include "spp/services/datalogger/datalogger.h"

extern const SPP_HalPort_t g_esp32HalPort;

/* ----------------------------------------------------------------
 * Sensor instances
 * ---------------------------------------------------------------- */

static ICM20948_t s_icm = {
    .spiDevIdx   = 0U, /* SPI device index 0 */
    .intPin      = 10U,
    .intIntrType = 1U, /* Rising edge */
    .intPull     = 0U, /* No pull */
};

static BMP390_t s_bmp = {
    .spiDevIdx   = 1U, /* SPI device index 1 */
    .intPin      = 17U,
    .intIntrType = 1U, /* Rising edge */
    .intPull     = 0U, /* No pull */
};

/* ----------------------------------------------------------------
 * SD card logger
 * ---------------------------------------------------------------- */

static const SPP_StorageInitCfg_t s_sdCfg = {
    .p_basePath          = "/sdcard",
    .spiHostId           = 1,
    .pinCs               = 9,
    .maxFiles            = 5U,
    .allocationUnitSize  = 16384U,
    .formatIfMountFailed = false,
};

static Datalogger_t s_logger = {
    .p_storageCfg = (void *)&s_sdCfg,
    .p_filePath   = "/sdcard/log.txt",
};

/* ----------------------------------------------------------------
 * app_main
 * ---------------------------------------------------------------- */

void app_main(void)
{
    (void)SPP_CORE_boot(&g_esp32HalPort);

    (void)SPP_HAL_spiBusInit();
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(0U)); /* ICM20948 */
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(1U)); /* BMP390   */

    /* Producers first — registration order sets dispatch order in callProducers(). */
    (void)SPP_SERVICES_register(&g_icm20948Module, &s_icm);
    (void)SPP_SERVICES_register(&g_bmp390Module,   &s_bmp);

    /* Consumer — subscribes to all packets at PRIO_LOW (deferred, never blocks sensors). */
    (void)SPP_SERVICES_register(&g_sdLoggerModule, &s_logger);

    for (;;)
    {
        SPP_SERVICES_callProducers();  /* each sensor: new data? → read → publish */
        SPP_SERVICES_callConsumers();  /* one consumer per call — SD write spread across iterations */
    }
}
