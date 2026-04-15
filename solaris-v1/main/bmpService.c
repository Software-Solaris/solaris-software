/**
 * @file bmpService.c
 * @brief BMP390 service implementation — reads pressure/temperature and logs packets.
 */

#include "bmpService.h"

#include "bmp390.h"
#include "gpio_int.h"
#include "dataloggerSD.h"
#include "macros_esp32.h"

#include "spp/services/databank.h"
#include "spp/services/db_flow.h"
#include "spp/services/log.h"
#include "spp/osal/task.h"
#include "spp/core/packet.h"
#include "spp/hal/storage.h"
#include "spp/core/types.h"

#include <string.h>

/* ----------------------------------------------------------------
 * Private constants
 * ---------------------------------------------------------------- */

static const char *s_bmpServiceLogTag = "BMP_SERVICE";

#define K_BMP_SERVICE_APID_DBG        (0x0101U)
#define K_BMP_SERVICE_TASK_PRIO       (5U)
#define K_BMP_SERVICE_TASK_DELAY_MS   (200U)
#define K_BMP_SERVICE_TASK_STACK_SIZE (4096U)
#define K_BMP_SERVICE_PAYLOAD_LEN     (12U)

#define K_BMP_SERVICE_LOG_FILE_PATH   "/sdcard/log.txt"
#define K_BMP_SERVICE_LOG_MAX_PACKETS (10U)

/* ----------------------------------------------------------------
 * Private state
 * ---------------------------------------------------------------- */

static void        *s_pSpi    = NULL;
static BMP390_Data_t s_bmpData;
static spp_uint16_t  s_seq    = 0U;

static SPP_StorageInitCfg_t s_sdCfg = {
    .p_basePath           = "/sdcard",
    .spiHostId            = K_ESP32_SPI_HOST,
    .pinCs                = K_ESP32_PIN_CS_SDC,
    .maxFiles             = 5U,
    .allocationUnitSize   = (16U * 1024U),
    .formatIfMountFailed  = false,
};

/* ----------------------------------------------------------------
 * Private: acquisition task
 * ---------------------------------------------------------------- */

static void BMP_ServiceTask(void *p_arg)
{
    Datalogger_t logger;
    retval_t     ret;
    spp_uint8_t  loggerActive = 0U;

    (void)p_arg;

    SPP_LOGI(s_bmpServiceLogTag, "Task start");

    ret = DATALOGGER_Init(&logger, (void *)&s_sdCfg, K_BMP_SERVICE_LOG_FILE_PATH);
    if (ret != SPP_OK)
    {
        SPP_LOGE(s_bmpServiceLogTag, "DATALOGGER_Init failed ret=%d", (int)ret);
    }
    else
    {
        loggerActive = 1U;
        SPP_LOGI(s_bmpServiceLogTag, "Datalogger ready");
    }

    for (;;)
    {
        SPP_Packet_t *p_packet   = NULL;
        SPP_Packet_t *p_packetRx = NULL;
        float         altitude    = 0.0f;
        float         pressure    = 0.0f;
        float         temperature = 0.0f;

        ret = BMP390_waitDrdy(&s_bmpData, 5000U);
        if (ret != SPP_OK)
        {
            SPP_LOGE(s_bmpServiceLogTag, "DRDY wait failed ret=%d", (int)ret);
            continue;
        }

        p_packet = SPP_Databank_getPacket();
        if (p_packet == NULL)
        {
            SPP_LOGI(s_bmpServiceLogTag, "No free packet");
            continue;
        }

        ret = BMP390_getAltitude(s_pSpi, &s_bmpData, &altitude, &pressure, &temperature);
        if (ret != SPP_OK)
        {
            SPP_LOGE(s_bmpServiceLogTag, "BMP390_getAltitude failed ret=%d -> return packet",
                     (int)ret);
            (void)SPP_Databank_returnPacket(p_packet);
            continue;
        }

        p_packet->primaryHeader.version    = K_SPP_PKT_VERSION;
        p_packet->primaryHeader.apid       = K_BMP_SERVICE_APID_DBG;
        p_packet->primaryHeader.seq        = s_seq++;
        p_packet->primaryHeader.payloadLen = K_BMP_SERVICE_PAYLOAD_LEN;

        p_packet->secondaryHeader.timestampMs = 0U;
        p_packet->secondaryHeader.dropCounter = 0U;

        p_packet->crc = 0U;

        memset(p_packet->payload, 0, K_SPP_PKT_PAYLOAD_MAX);
        memcpy(&p_packet->payload[0], &altitude,    sizeof(float));
        memcpy(&p_packet->payload[4], &pressure,    sizeof(float));
        memcpy(&p_packet->payload[8], &temperature, sizeof(float));

        ret = SPP_DbFlow_pushReady(p_packet);
        if (ret != SPP_OK)
        {
            SPP_LOGE(s_bmpServiceLogTag, "SPP_DbFlow_pushReady failed ret=%d -> return packet",
                     (int)ret);
            (void)SPP_Databank_returnPacket(p_packet);
            continue;
        }

        ret = SPP_DbFlow_popReady(&p_packetRx);
        if ((ret != SPP_OK) || (p_packetRx == NULL))
        {
            SPP_LOGE(s_bmpServiceLogTag, "SPP_DbFlow_popReady failed ret=%d pkt_rx=%p",
                     (int)ret, (void *)p_packetRx);
            continue;
        }

        if (loggerActive == 1U)
        {
            ret = DATALOGGER_LogPacket(&logger, p_packetRx);
            if (ret != SPP_OK)
            {
                SPP_LOGE(s_bmpServiceLogTag, "DATALOGGER_LogPacket failed ret=%d", (int)ret);
            }

            if (logger.logged_packets >= K_BMP_SERVICE_LOG_MAX_PACKETS)
            {
                ret = DATALOGGER_Flush(&logger);
                if (ret != SPP_OK)
                {
                    SPP_LOGE(s_bmpServiceLogTag, "DATALOGGER_Flush failed ret=%d", (int)ret);
                }

                ret = DATALOGGER_Deinit(&logger);
                if (ret != SPP_OK)
                {
                    SPP_LOGE(s_bmpServiceLogTag, "DATALOGGER_Deinit failed ret=%d", (int)ret);
                }
                else
                {
                    SPP_LOGI(s_bmpServiceLogTag, "Datalogger finished after %u packets",
                             (unsigned)K_BMP_SERVICE_LOG_MAX_PACKETS);
                }

                loggerActive = 0U;
            }
        }

        ret = SPP_Databank_returnPacket(p_packetRx);
        if (ret != SPP_OK)
        {
            SPP_LOGE(s_bmpServiceLogTag, "SPP_Databank_returnPacket failed ret=%d", (int)ret);
            continue;
        }

        SPP_Osal_taskDelayMs(K_BMP_SERVICE_TASK_DELAY_MS);
    }
}

/* ----------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------- */

retval_t BMP_ServiceInit(void *p_spiBmp)
{
    retval_t ret;

    if (p_spiBmp == NULL)
    {
        return SPP_ERROR_NULL_POINTER;
    }

    s_pSpi = p_spiBmp;

    s_bmpData.intPin      = (spp_uint32_t)INT_GPIO;
    s_bmpData.intIntrType = 1U;
    s_bmpData.intPull     = 0U;

    BMP390_init(&s_bmpData);

    ret = BMP390_auxConfig(s_pSpi);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = BMP390_prepareMeasure(s_pSpi);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = BMP390_intEnableDrdy(s_pSpi);
    if (ret != SPP_OK)
    {
        return ret;
    }

    return SPP_OK;
}

retval_t BMP_ServiceStart(void)
{
    void *p_taskHandle;

    p_taskHandle = SPP_Osal_taskCreate(
        BMP_ServiceTask,
        "bmp_service",
        K_BMP_SERVICE_TASK_STACK_SIZE,
        NULL,
        K_BMP_SERVICE_TASK_PRIO);

    if (p_taskHandle == NULL)
    {
        SPP_LOGE(s_bmpServiceLogTag, "TaskCreate failed");
        return SPP_ERROR;
    }

    SPP_LOGI(s_bmpServiceLogTag, "Task created");
    return SPP_OK;
}
