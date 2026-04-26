#include "spp/spp.h"
#include "spp/services/bmp390/bmp390.h"
#include "spp/services/icm20948/icm20948.h"
/* #include "spp/services/datalogger/datalogger.h" */

#include <stdio.h>
#include <string.h>

/* ----------------------------------------------------------------
 * HAL port (provided by the spp_ports component)
 * ---------------------------------------------------------------- */

extern const SPP_HalPort_t g_esp32HalPort;

/* ----------------------------------------------------------------
 * Private constants
 * ---------------------------------------------------------------- */

static const char *const k_tag = "MAIN";

/* ----------------------------------------------------------------
 * BMP390 service
 * ---------------------------------------------------------------- */

static BMP390_ServiceCtx_t s_bmpCtx;
static const BMP390_ServiceCfg_t s_bmpCfg = {
    .spiDevIdx = 1U,   /* BMP390 = SPI device index 1 */
    .intPin = 17U,     /* DRDY GPIO */
    .intIntrType = 1U, /* Rising edge */
    .intPull = 0U,     /* No pull */
};

/* ----------------------------------------------------------------
 * ICM20948 service
 * ---------------------------------------------------------------- */

static ICM20948_ServiceCtx_t s_icmCtx;
static const ICM20948_ServiceCfg_t s_icmCfg = {
    .spiDevIdx = 0U,   /* ICM20948 = SPI device index 0 */
    .intPin = 10U,     /* INT GPIO */
    .intIntrType = 1U, /* Rising edge */
    .intPull = 0U,     /* No pull */
};

/* ----------------------------------------------------------------
 * SD card logger (disabled)
 * ---------------------------------------------------------------- */

/*
static Datalogger_t s_logger;

static const SPP_StorageInitCfg_t s_storageCfg = {
    .p_basePath = "/sdcard",
    .spiHostId = 1,
    .pinCs = 9,
    .maxFiles = 5U,
    .allocationUnitSize = 16384U,
    .formatIfMountFailed = false,
};

#define K_SD_FLUSH_EVERY (20U)

static void sdLogHandler(const SPP_Packet_t *p_packet, void *p_ctx)
{
    Datalogger_t *p_log = (Datalogger_t *)p_ctx;
    (void)SPP_SERVICES_DATALOGGER_logPacket(p_log, p_packet);

    if ((p_log->logged_packets % K_SD_FLUSH_EVERY) == 0U)
    {
        (void)SPP_SERVICES_DATALOGGER_flush(p_log);
    }
}
*/

/* ----------------------------------------------------------------
 * SPP_LOG → pub/sub bridge
 *
 * Registers as the log output function.  Every SPP_LOG* call formats
 * a K_SPP_APID_LOG packet and publishes it.  The SD card subscriber
 * receives it and writes the string directly to the log file.
 * ---------------------------------------------------------------- */

static spp_uint16_t s_logSeq = 0U;
static spp_bool_t s_logBusy = false;

static void logPubSubOutput(const char *p_tag, SPP_LogLevel_t level, const char *p_message)
{
    static const char k_lvl[] = "?EWID V";
    char lvlChar = k_lvl[(unsigned)level < sizeof(k_lvl) ? (unsigned)level : 0U];

    printf("[%c] %s: %s\n", lvlChar, p_tag, p_message);

    if (s_logBusy)
    {
        return;
    }
    s_logBusy = true;

    SPP_Packet_t *p_pkt = SPP_SERVICES_DATABANK_getPacket();
    if (p_pkt != NULL)
    {
        char buf[K_SPP_PKT_PAYLOAD_MAX];
        int n = snprintf(buf, sizeof(buf), "[%c] %s: %s", lvlChar, p_tag, p_message);
        spp_uint16_t len =
            (n > 0 && n < (int)sizeof(buf)) ? (spp_uint16_t)(n + 1U) : (spp_uint16_t)sizeof(buf);

        (void)SPP_SERVICES_DATABANK_packetData(p_pkt, K_SPP_APID_LOG, s_logSeq++, buf, len);
        (void)SPP_SERVICES_PUBSUB_publish(p_pkt);
    }

    s_logBusy = false;
}

/* ----------------------------------------------------------------
 * app_main
 * ---------------------------------------------------------------- */

void app_main(void)
{
    (void)SPP_CORE_boot(&g_esp32HalPort);

    (void)SPP_HAL_spiBusInit();
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(0U));
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(1U));

    /* 4. SD card logger (disabled). */
    /*
    if (SPP_SERVICES_DATALOGGER_init(&s_logger, (void *)&s_storageCfg, "/sdcard/log.txt") !=
        K_SPP_OK)
    {
        printf("[W] app_main: SD card unavailable — continuing without logging\n");
    }
    (void)SPP_SERVICES_PUBSUB_subscribe(K_SPP_APID_ALL, sdLogHandler, &s_logger);
    */

    /* 5. Register, init and start BMP390 + ICM20948. */
    (void)SPP_SERVICES_register(&g_icm20948ServiceDesc, &s_icmCtx, &s_icmCfg);
    (void)SPP_SERVICES_register(&g_bmp390ServiceDesc, &s_bmpCtx, &s_bmpCfg);
    (void)SPP_SERVICES_initAll();
    (void)SPP_SERVICES_startAll();

    SPP_LOGI(k_tag, "Services ready — entering superloop");

    /* ----------------------------------------------------------------
     * Superloop
     * ---------------------------------------------------------------- */
    for (;;)
    {
        SPP_SERVICES_callProducers();
        SPP_SERVICES_callConsumers(); /* one per call — spreads SD writes across iterations */
    }
}
