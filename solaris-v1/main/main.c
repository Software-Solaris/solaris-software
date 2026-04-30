/**
 * @file main.c
 * @brief Solaris v1 application entry point — bare-metal superloop.
 *
 * Data flow:
 *   ISR sets drdyFlag
 *     → SPP_SERVICES_pollAll() calls each module's serviceTask
 *       → serviceTask reads sensor, builds packet, calls publish()
 *         → CRITICAL subscribers run synchronously inside publish()
 *         → other subscribers dispatched one-per-call via tick()
 */

#include "spp/spp.h"
#include "spp/services/bmp390/bmp390.h"
#include "spp/services/icm20948/icm20948.h"

extern const SPP_HalPort_t g_esp32HalPort;

/* ----------------------------------------------------------------
 * Sensor instances — fill in config fields, runtime state set by init
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
 * SD card logger — disabled; to enable:
 *   1. Include "spp/services/datalogger/datalogger.h"
 *   2. Declare: static Datalogger_t s_logger = { .p_storageCfg = ..., .p_filePath = ... };
 *   3. Add: SPP_SERVICES_register(&g_sdLoggerModule, &s_logger);
 * ---------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * app_main
 * ---------------------------------------------------------------- */

void app_main(void)
{
    (void)SPP_CORE_boot(&g_esp32HalPort);

    (void)SPP_HAL_spiBusInit();
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(0U)); /* ICM20948 */
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(1U)); /* BMP390   */

    (void)SPP_SERVICES_register(&g_icm20948Module, &s_icm);
    (void)SPP_SERVICES_register(&g_bmp390Module,   &s_bmp);

    for (;;)
    {
        SPP_SERVICES_pollAll();
        SPP_SERVICES_PUBSUB_tick();
    }
}
