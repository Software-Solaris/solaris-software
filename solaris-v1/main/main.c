#include "spp/spp.h"
#include "spp/services/bmp390/bmp390.h"
#include "spp/services/icm20948/icm20948.h"

extern const SPP_HalPort_t g_esp32HalPort;

static ICM20948_t s_icm = {
    .spiDevIdx = 0U,
    .intPin = 10U,
    .intIntrType = 1U,
    .intPull = 0U,
};

static BMP390_t s_bmp = {
    .spiDevIdx = 1U,
    .intPin = 17U,
    .intIntrType = 1U,
    .intPull = 0U,
};

static const SPP_StorageInitCfg_t s_sdCfg = {
    .p_basePath = "/sdcard",
    .spiHostId = 1,
    .pinCs = 9,
    .maxFiles = 5U,
    .allocationUnitSize = 16384U,
    .formatIfMountFailed = false,
};

static Datalogger_t s_logger = {
    .p_storageCfg = (void *)&s_sdCfg,
    .p_filePath = "/sdcard/log.txt",
};

void app_main(void)
{
    (void)SPP_CORE_boot(&g_esp32HalPort);

    (void)SPP_HAL_spiBusInit();
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(0U));
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(1U));

    (void)SPP_SERVICES_register(&g_icm20948Module, &s_icm);
    (void)SPP_SERVICES_register(&g_bmp390Module, &s_bmp);
    (void)SPP_SERVICES_register(&g_sdLoggerModule, &s_logger);

    for (;;)
    {
        SPP_SERVICES_callProducers();
        SPP_SERVICES_callConsumers(); /* one per call — spreads SD writes across iterations */
    }
}
