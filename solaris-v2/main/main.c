#include "spp/core/returnTypes.h"
#include "spp/spp.h"
#include "spp/services/bmp390/bmp390.h"
#include "spp/services/icm20948/icm20948.h"
#include "spp/services/pubsub/pubsub.h"
#include "spp/services/service.h"
#include "spp/hal/hal.h"
/* #include "spp/services/datalogger/datalogger.h" */

/* Bring external variable from the HAL of ESP32 platform */
extern const SPP_HalPort_t g_esp32HalPorts;

// static ICM20948_t s_icm = {
//     .spiDevIdx = 0U,
//     .intPin = 10U,
//     .intIntrType = 1U,
//     .intPull = 0U,
// };

// static const SPP_StorageInitCfg_t s_sdCfg = {
//     .p_basePath = "/sdcard",
//     .spiHostId = 1,
//     .pinCs = 9,
//     .maxFiles = 5U,
//     .allocationUnitSize = 16384U,
//     .formatIfMountFailed = false,
// };

// static Datalogger_t s_logger = {
//     .p_storageCfg = (void *)&s_sdCfg,
//     .p_filePath = "/sdcard/log.txt",
// };

// static uint32_t s_rxCount = 0U;

// static void debugSubscriber(const SPP_Packet_t *p_packet, void *p_ctx)
// {
//     (void)p_ctx;
//     s_rxCount++;
//     if (s_rxCount % 50U == 0U)
//     {
//         printf("[DEBUG] rx=%u apid=0x%04X queue=%u overflow=%u\n", (unsigned)s_rxCount,
//                (unsigned)p_packet->primaryHeader.apid, (unsigned)SPP_SERVICES_PUBSUB_queueDepth(),
//                (unsigned)SPP_SERVICES_PUBSUB_overflowCount(K_SPP_APID_ALL));
//     }
// }

void app_main(void)
{
    /*  Set the HAL ports to the platform being used (ESP32S3)
        and do all the init required to get the core stack running */
    SPP_RetVal_t ret = SPP_HAL_init(&g_esp32HalPorts);
    if (ret != K_SPP_OK)
    {
        /* Some error happened doing the HAL initialization */
        return;
    }

    // Register the BMP390 service
    const SPP_SERVICE_ProducerContract_t *p_bmpProducerContract = SPP_SERVICES_BMP390_getProducerContract();
    if (p_bmpProducerContract == NULL)
    {
        return;
    }
    else
    {
        ret = SPP_SERVICES_PUBSUB_registerProducer(p_bmpProducerContract);
        if (ret != K_SPP_OK)
        {
            return;
        }
    }

    //Register ICM20948 producer
    const SPP_SERVICE_ProducerContract_t *p_icmProducerContract = SPP_SERVICES_ICM20948_getProducerContract();
    if (p_icmProducerContract == NULL)
    {
        return;
    }
    else
    {
        ret = SPP_SERVICES_PUBSUB_registerProducer(p_icmProducerContract);
        if (ret != K_SPP_OK)
        {
            return;
        }
    }

    /* Init the SPP core software stack */
    ret = SPP_CORE_init();
    if (ret != K_SPP_OK)
    {
        /* Some error happened doing the core initialization */
        return;
    }

    // Init this in another place
    // (void)SPP_HAL_SPI_deviceInit(SPP_HAL_SPI_getHandle(0U));
    // (void)SPP_HAL_SPI_deviceInit(SPP_HAL_SPI_getHandle(1U));

    // (void)SPP_SERVICES_register(&g_icm20948Module, &s_icm);
    // (void)SPP_SERVICES_register(&g_bmp390Module, &s_bmp);
    // (void)SPP_SERVICES_register(&g_sdLoggerModule, &s_logger);

    for (;;)
    {
        // SPP_SERVICES_callProducers();
        // SPP_SERVICES_callConsumers(); /* one per call — spreads SD writes across iterations */
    }
}
