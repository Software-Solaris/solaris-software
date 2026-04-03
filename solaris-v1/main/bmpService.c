#include "bmpService.h"

#include "bmp390.h"
#include "gpio_int.h"
#include "dataloggerSD.h"
#include "storage.h"
#include "spi.h"
#include "macros_esp.h"

#include "services/databank/databank.h"
#include "services/db_flow/db_flow.h"

#include "osal/task.h"
#include "spp_log.h"

#include <string.h>

/* ----------------------------------------------------------------
 * Private constants
 * ---------------------------------------------------------------- */

static const char *s_bmpServiceLogTag = "BMP_SERVICE";

#define K_BMP_SERVICE_APID_DBG        0x0101U
#define K_BMP_SERVICE_TASK_PRIO       5U
#define K_BMP_SERVICE_TASK_DELAY_MS   200U
#define K_BMP_SERVICE_TASK_STACK_SIZE 4096U
#define K_BMP_SERVICE_PAYLOAD_LEN     12U

#define K_BMP_SERVICE_LOG_FILE_PATH   "/sdcard/log.txt"
#define K_BMP_SERVICE_LOG_MAX_PACKETS 10U

/* ----------------------------------------------------------------
 * Private state
 * ---------------------------------------------------------------- */

static void *s_pSpi = NULL;
static BMP390_Data_t s_bmpData;
static spp_uint16_t s_seq = 0U;

static SPP_Storage_InitCfg s_sdCfg = {.p_base_path = "/sdcard",
                                      .spi_host_id = USED_HOST,
                                      .pin_cs = CS_PIN_SDC,
                                      .max_files = 5U,
                                      .allocation_unit_size = (16U * 1024U),
                                      .format_if_mount_failed = false};

static void BMP_ServiceTask(void *p_arg)
{
    Datalogger_t logger;
    retval_t ret;
    uint8_t logger_active = 0U;

    (void)p_arg;

    SPP_LOGI(s_bmpServiceLogTag, "Task start");

    ret = DATALOGGER_Init(&logger, (void *)&s_sdCfg, K_BMP_SERVICE_LOG_FILE_PATH);
    if (ret != SPP_OK)
    {
        SPP_LOGE(s_bmpServiceLogTag, "DATALOGGER_Init failed ret=%d", (int)ret);
    }
    else
    {
        logger_active = 1U;
        SPP_LOGI(s_bmpServiceLogTag, "Datalogger ready");
    }

    for (;;)
    {
        spp_packet_t *p_packet;
        float altitude = 0.0f;
        float pressure = 0.0f;
        float temperature = 0.0f;
        spp_packet_t *p_packetRx = NULL;

        ret = BMP390_waitDrdy(&s_bmpData, 5000U);
        if (ret != SPP_OK)
        {
            SPP_LOGE(s_bmpServiceLogTag, "DRDY wait failed ret=%d", (int)ret);
            continue;
        }

        p_packet = SPP_DATABANK_getPacket();
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
            (void)SPP_DATABANK_returnPacket(p_packet);
            continue;
        }

        p_packet->primary_header.version = SPP_PKT_VERSION;
        p_packet->primary_header.apid = K_BMP_SERVICE_APID_DBG;
        p_packet->primary_header.seq = s_seq++;
        p_packet->primary_header.payload_len = K_BMP_SERVICE_PAYLOAD_LEN;

        p_packet->secondary_header.timestamp_ms = 0U;
        p_packet->secondary_header.drop_counter = 0U;

        p_packet->crc = 0U;

        memset(p_packet->payload, 0, SPP_PKT_PAYLOAD_MAX);
        memcpy(&p_packet->payload[0], &altitude, sizeof(float));
        memcpy(&p_packet->payload[4], &pressure, sizeof(float));
        memcpy(&p_packet->payload[8], &temperature, sizeof(float));

        ret = DB_FLOW_PushReady(p_packet);
        if (ret != SPP_OK)
        {
            SPP_LOGE(s_bmpServiceLogTag, "DB_FLOW_PushReady failed ret=%d -> return packet",
                     (int)ret);
            (void)SPP_DATABANK_returnPacket(p_packet);
            continue;
        }

        ret = DB_FLOW_PopReady(&p_packetRx);
        if ((ret != SPP_OK) || (p_packetRx == NULL))
        {
            SPP_LOGE(s_bmpServiceLogTag, "DB_FLOW_PopReady failed ret=%d pkt_rx=%p", (int)ret,
                     (void *)p_packetRx);
            continue;
        }

        if (logger_active == 1U)
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

                logger_active = 0U;
            }
        }

        ret = SPP_DATABANK_returnPacket(p_packetRx);
        if (ret != SPP_OK)
        {
            SPP_LOGE(s_bmpServiceLogTag, "SPP_DATABANK_returnPacket failed ret=%d", (int)ret);
            continue;
        }

        SPP_OSAL_TaskDelay(K_BMP_SERVICE_TASK_DELAY_MS);
    }
}

retval_t BMP_ServiceInit(void *p_spi_bmp)
{
    retval_t ret;

    if (p_spi_bmp == NULL)
    {
        return SPP_ERROR_NULL_POINTER;
    }

    s_pSpi = p_spi_bmp;

    s_bmpData.intPin = (spp_uint32_t)INT_GPIO;
    s_bmpData.intIntrType = 1U;
    s_bmpData.intPull = 0U;

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
    void *p_storage;
    void *p_taskHandle;

    p_storage = SPP_OSAL_GetTaskStorage();
    if (p_storage == NULL)
    {
        SPP_LOGE(s_bmpServiceLogTag, "No task storage");
        return SPP_ERROR;
    }

    p_taskHandle =
        SPP_OSAL_TaskCreate((void *)BMP_ServiceTask, "bmp_service", K_BMP_SERVICE_TASK_STACK_SIZE,
                            NULL, K_BMP_SERVICE_TASK_PRIO, p_storage);

    if (p_taskHandle == NULL)
    {
        SPP_LOGE(s_bmpServiceLogTag, "TaskCreate failed");
        return SPP_ERROR;
    }

    SPP_LOGI(s_bmpServiceLogTag, "Task created");
    return SPP_OK;
}