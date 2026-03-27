#include "bmpService.h"

#include "bmp390.h"
#include "gpio_int.h"

#include "services/databank/databank.h"
#include "services/db_flow/db_flow.h"

#include "osal/task.h"
#include "spp_log.h"

#include <string.h>

/* ----------------------------------------------------------------
 * Private constants
 * ---------------------------------------------------------------- */

/**
 * @brief Log tag used by the BMP390 service.
 */
static const char *s_bmpServiceLogTag = "BMP_SERVICE";

/**
 * @brief Application Process Identifier used for BMP debug packets.
 */
#define K_BMP_SERVICE_APID_DBG             0x0101U

/**
 * @brief Priority of the BMP service task.
 */
#define K_BMP_SERVICE_TASK_PRIO            5U

/**
 * @brief Delay between iterations of the BMP service task, in milliseconds.
 */
#define K_BMP_SERVICE_TASK_DELAY_MS        200U

/**
 * @brief Stack size of the BMP service task.
 */
#define K_BMP_SERVICE_TASK_STACK_SIZE      4096U

/**
 * @brief Payload length in bytes for altitude, pressure and temperature floats.
 */
#define K_BMP_SERVICE_PAYLOAD_LEN          12U

/* ----------------------------------------------------------------
 * Private state
 * ---------------------------------------------------------------- */

/**
 * @brief SPI handler associated with the BMP sensor.
 */
static void *s_pSpi = NULL;

/**
 * @brief BMP390 service context.
 */
static BMP390_Data_t s_bmpData;

/**
 * @brief Packet sequence counter used by the BMP service.
 */
static spp_uint16_t s_seq = 0U;

/* ----------------------------------------------------------------
 * Private helpers
 * ---------------------------------------------------------------- */

/**
 * @brief Logs the basic contents of a packet header.
 *
 * @param[in] p_prefix Prefix string to identify the log source.
 * @param[in] p_packet Pointer to the packet to log.
 */
static void BMP_ServiceLogPacketBasic(const char *p_prefix, const spp_packet_t *p_packet)
{
    if (p_packet == NULL)
    {
        SPP_LOGE(s_bmpServiceLogTag, "%s pkt=NULL", p_prefix);
        return;
    }

    SPP_LOGI(s_bmpServiceLogTag,
             "%s pkt=%p ver=%u apid=0x%04X seq=%u len=%u crc=%u",
             p_prefix,
             (void *)p_packet,
             (unsigned)p_packet->primary_header.version,
             (unsigned)p_packet->primary_header.apid,
             (unsigned)p_packet->primary_header.seq,
             (unsigned)p_packet->primary_header.payload_len,
             (unsigned)p_packet->crc);
}

/**
 * @brief Logs the payload of a packet as three float values.
 *
 * The payload is interpreted as:
 * - altitude at payload[0]
 * - pressure at payload[4]
 * - temperature at payload[8]
 *
 * @param[in] p_prefix Prefix string to identify the log source.
 * @param[in] p_packet Pointer to the packet to log.
 */
static void BMP_ServiceLogPacketPayloadFloats(const char *p_prefix, const spp_packet_t *p_packet)
{
    float altitude = 0.0f;
    float pressure = 0.0f;
    float temperature = 0.0f;

    if (p_packet == NULL)
    {
        return;
    }

    if (p_packet->primary_header.payload_len < K_BMP_SERVICE_PAYLOAD_LEN)
    {
        SPP_LOGI(s_bmpServiceLogTag,
                 "%s payload too small len=%u",
                 p_prefix,
                 (unsigned)p_packet->primary_header.payload_len);
        return;
    }

    memcpy(&altitude, &p_packet->payload[0], sizeof(float));
    memcpy(&pressure, &p_packet->payload[4], sizeof(float));
    memcpy(&temperature, &p_packet->payload[8], sizeof(float));

    SPP_LOGI(s_bmpServiceLogTag,
             "%s payload alt=%.2f p=%.2f t=%.2f",
             p_prefix,
             altitude,
             pressure,
             temperature);
}

/**
 * @brief BMP390 service task.
 *
 * This task waits for the BMP390 data-ready interrupt, acquires altitude,
 * pressure and temperature from the sensor, stores them in a packet,
 * publishes the packet to the ready flow, then simulates a consumer that
 * pops the packet and returns it to the databank.
 *
 * @param[in] p_arg Unused task argument.
 */
static void BMP_ServiceTask(void *p_arg)
{
    (void)p_arg;

    SPP_LOGI(s_bmpServiceLogTag, "Task start");

    for (;;)
    {
        retval_t ret;
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

        SPP_LOGI(s_bmpServiceLogTag, "DRDY received");

        SPP_LOGI(s_bmpServiceLogTag, "Requesting free packet...");
        p_packet = SPP_DATABANK_getPacket();
        if (p_packet == NULL)
        {
            SPP_LOGI(s_bmpServiceLogTag, "No free packet");
            continue;
        }

        SPP_LOGI(s_bmpServiceLogTag, "Got free packet ptr=%p", (void *)p_packet);
        BMP_ServiceLogPacketBasic("FREE_PKT", p_packet);

        ret = BMP390_getAltitude(s_pSpi, &s_bmpData, &altitude, &pressure, &temperature);
        if (ret != SPP_OK)
        {
            SPP_LOGE(s_bmpServiceLogTag,
                     "BMP390_getAltitude failed ret=%d -> return packet",
                     (int)ret);
            (void)SPP_DATABANK_returnPacket(p_packet);
            continue;
        }

        SPP_LOGI(s_bmpServiceLogTag,
                 "BMP read alt=%.2f p=%.2f t=%.2f",
                 altitude,
                 pressure,
                 temperature);

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

        BMP_ServiceLogPacketBasic("FILLED", p_packet);
        BMP_ServiceLogPacketPayloadFloats("FILLED", p_packet);

        SPP_LOGI(s_bmpServiceLogTag, "Publishing to DB_FLOW...");
        ret = DB_FLOW_PushReady(p_packet);
        if (ret != SPP_OK)
        {
            SPP_LOGE(s_bmpServiceLogTag,
                     "DB_FLOW_PushReady failed ret=%d -> return packet",
                     (int)ret);
            (void)SPP_DATABANK_returnPacket(p_packet);
            continue;
        }

        SPP_LOGI(s_bmpServiceLogTag,
                 "Published ok ready=%lu",
                 (unsigned long)DB_FLOW_ReadyCount());

        SPP_LOGI(s_bmpServiceLogTag, "Consumer pop...");
        ret = DB_FLOW_PopReady(&p_packetRx);
        if ((ret != SPP_OK) || (p_packetRx == NULL))
        {
            SPP_LOGE(s_bmpServiceLogTag,
                     "DB_FLOW_PopReady failed ret=%d pkt_rx=%p",
                     (int)ret,
                     (void *)p_packetRx);
            continue;
        }

        SPP_LOGI(s_bmpServiceLogTag,
                 "Consumer got ptr=%p ready=%lu",
                 (void *)p_packetRx,
                 (unsigned long)DB_FLOW_ReadyCount());

        BMP_ServiceLogPacketBasic("RECEIVED", p_packetRx);
        BMP_ServiceLogPacketPayloadFloats("RECEIVED", p_packetRx);

        SPP_LOGI(s_bmpServiceLogTag, "Returning packet to databank...");
        ret = SPP_DATABANK_returnPacket(p_packetRx);
        if (ret != SPP_OK)
        {
            SPP_LOGE(s_bmpServiceLogTag,
                     "SPP_DATABANK_returnPacket failed ret=%d",
                     (int)ret);
            continue;
        }

        BMP_ServiceLogPacketBasic("AFTER_RETURN", p_packetRx);
        BMP_ServiceLogPacketPayloadFloats("AFTER_RETURN", p_packetRx);

        SPP_OSAL_TaskDelay(K_BMP_SERVICE_TASK_DELAY_MS);
    }
}

/* ----------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------- */

/** @copydoc BMP_ServiceInit */
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

/**
 * @brief Starts the BMP service task.
 *
 * This function allocates task storage and creates the BMP service task.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
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

    p_taskHandle = SPP_OSAL_TaskCreate(
        (void *)BMP_ServiceTask,
        "bmp_service",
        K_BMP_SERVICE_TASK_STACK_SIZE,
        NULL,
        K_BMP_SERVICE_TASK_PRIO,
        p_storage);

    if (p_taskHandle == NULL)
    {
        SPP_LOGE(s_bmpServiceLogTag, "TaskCreate failed");
        return SPP_ERROR;
    }

    SPP_LOGI(s_bmpServiceLogTag, "Task created");
    return SPP_OK;
}