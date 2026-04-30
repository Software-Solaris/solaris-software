#include "spp/spp.h"
#include "spp/services/bmp390/bmp390.h"
#include "spp/services/icm20948/icm20948.h"
#include "spp/services/datalogger/datalogger.h"
#include "spp/services/pubsub/pubsub.h"
#include <stdio.h>

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

static uint32_t s_rxCount = 0U;

static void debugSubscriber(const SPP_Packet_t *p_packet, void *p_ctx)
{
    (void)p_ctx;
    s_rxCount++;
    if (s_rxCount % 50U == 0U)
    {
        printf("[DEBUG] rx=%u apid=0x%04X queue=%u overflow=%u\n", (unsigned)s_rxCount,
               (unsigned)p_packet->primaryHeader.apid, (unsigned)SPP_SERVICES_PUBSUB_queueDepth(),
               (unsigned)SPP_SERVICES_PUBSUB_overflowCount(K_SPP_APID_ALL));
    }
}

void app_main(void)
{
    (void)SPP_CORE_boot(&g_esp32HalPort);

    (void)SPP_HAL_spiBusInit();
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(0U));
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(1U));

    (void)SPP_SERVICES_register(&g_icm20948Module, &s_icm);
    (void)SPP_SERVICES_register(&g_bmp390Module, &s_bmp);
    (void)SPP_SERVICES_register(&g_sdLoggerModule, &s_logger);

    SPP_SERVICES_PUBSUB_subscribe(K_SPP_APID_ALL, K_SPP_PUBSUB_PRIO_LOW, debugSubscriber, NULL);

    for (;;)
    {
        SPP_SERVICES_callProducers();
        SPP_SERVICES_callConsumers(); /* one per call — spreads SD writes across iterations */
    }
}
