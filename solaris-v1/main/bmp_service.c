#include "bmp_service.h"

#include "bmp390.h"
#include "gpio_int.h"

#include "services/databank/databank.h"
#include "services/db_flow/db_flow.h"

#include "osal/task.h"
#include "spp_log.h"

#include <string.h>

static const char* TAG = "BMP_SERVICE";

#define BMP_APID_DBG         0x0101
#define BMP_TASK_PRIO        5
#define BMP_TASK_DELAY_MS    200

static void* s_spi = NULL;
static bmp_data_t s_bmp;
static spp_uint16_t s_seq = 0;

static void log_packet_basic(const char* prefix, const spp_packet_t* pkt)
{
    if (pkt == NULL) {
        SPP_LOGE(TAG, "%s pkt=NULL", prefix);
        return;
    }

    SPP_LOGI(TAG, "%s pkt=%p ver=%u apid=0x%04X seq=%u len=%u crc=%u",
             prefix,
             (void*)pkt,
             (unsigned)pkt->primary_header.version,
             (unsigned)pkt->primary_header.apid,
             (unsigned)pkt->primary_header.seq,
             (unsigned)pkt->primary_header.payload_len,
             (unsigned)pkt->crc);
}

static void log_packet_payload_floats(const char* prefix, const spp_packet_t* pkt)
{
    if (pkt == NULL) return;

    if (pkt->primary_header.payload_len < 12u) {
        SPP_LOGI(TAG, "%s payload too small len=%u", prefix, (unsigned)pkt->primary_header.payload_len);
        return;
    }

    float alt = 0.0f;
    float p   = 0.0f;
    float t   = 0.0f;

    memcpy(&alt, &pkt->payload[0],  sizeof(float));
    memcpy(&p,   &pkt->payload[4],  sizeof(float));
    memcpy(&t,   &pkt->payload[8],  sizeof(float));

    SPP_LOGI(TAG, "%s payload alt=%.2f p=%.2f t=%.2f", prefix, alt, p, t);
}

static void bmp_service_task(void* arg)
{
    (void)arg;

    SPP_LOGI(TAG, "Task start");

    for (;;)
    {
        // Wait DRDY
        retval_t ret = bmp390_wait_drdy(&s_bmp, 5000);
        if (ret != SPP_OK) {
            SPP_LOGE(TAG, "DRDY wait failed ret=%d", (int)ret);
            continue;
        }
        SPP_LOGI(TAG, "DRDY received");

        // Get free packet
        SPP_LOGI(TAG, "Requesting free packet...");
        spp_packet_t* pkt = SPP_DATABANK_getPacket();
        if (pkt == NULL) {
            SPP_LOGI(TAG, "No free packet");
            continue;
        }
        SPP_LOGI(TAG, "Got free packet ptr=%p", (void*)pkt);
        log_packet_basic("FREE_PKT", pkt);

        // Read BMP measures
        float alt = 0.0f;
        float p   = 0.0f;
        float t   = 0.0f;

        ret = bmp390_get_altitude(s_spi, &s_bmp, &alt, &p, &t);
        if (ret != SPP_OK) {
            SPP_LOGE(TAG, "bmp390_get_altitude failed ret=%d -> return packet", (int)ret);
            (void)SPP_DATABANK_returnPacket(pkt);
            continue;
        }

        SPP_LOGI(TAG, "BMP read alt=%.2f p=%.2f t=%.2f", alt, p, t);

        // Fill packet header (CRC=0)
        pkt->primary_header.version     = SPP_PKT_VERSION;
        pkt->primary_header.apid        = BMP_APID_DBG;
        pkt->primary_header.seq         = s_seq++;
        pkt->primary_header.payload_len = 12u;

        // Secondary header unused for debug
        pkt->secondary_header.timestamp_ms = 0;
        pkt->secondary_header.drop_counter = 0;

        pkt->crc = 0;

        // Fill payload with floats (no encoding)
        memset(pkt->payload, 0, SPP_PKT_PAYLOAD_MAX);
        memcpy(&pkt->payload[0], &alt, sizeof(float));
        memcpy(&pkt->payload[4], &p,   sizeof(float));
        memcpy(&pkt->payload[8], &t,   sizeof(float));

        log_packet_basic("FILLED", pkt);
        log_packet_payload_floats("FILLED", pkt);

        // Publish to ready FIFO
        SPP_LOGI(TAG, "Publishing to DB_FLOW...");
        ret = DB_FLOW_PushReady(pkt);
        if (ret != SPP_OK) {
            SPP_LOGE(TAG, "DB_FLOW_PushReady failed ret=%d -> return packet", (int)ret);
            (void)SPP_DATABANK_returnPacket(pkt);
            continue;
        }
        SPP_LOGI(TAG, "Published ok ready=%lu", (unsigned long)DB_FLOW_ReadyCount());

        // Simulated consumer pop
        SPP_LOGI(TAG, "Consumer pop...");
        spp_packet_t* pkt_rx = NULL;
        ret = DB_FLOW_PopReady(&pkt_rx);
        if ((ret != SPP_OK) || (pkt_rx == NULL)) {
            SPP_LOGE(TAG, "DB_FLOW_PopReady failed ret=%d pkt_rx=%p", (int)ret, (void*)pkt_rx);
            continue;
        }

        SPP_LOGI(TAG, "Consumer got ptr=%p ready=%lu", (void*)pkt_rx, (unsigned long)DB_FLOW_ReadyCount());
        log_packet_basic("RECEIVED", pkt_rx);
        log_packet_payload_floats("RECEIVED", pkt_rx);

        // Return to databank (databank clears it)
        SPP_LOGI(TAG, "Returning packet to databank...");
        ret = SPP_DATABANK_returnPacket(pkt_rx);
        if (ret != SPP_OK) {
            SPP_LOGE(TAG, "SPP_DATABANK_returnPacket failed ret=%d", (int)ret);
            continue;
        }

        // Databank does memset(pkt, 0)
        log_packet_basic("AFTER_RETURN", pkt_rx);
        log_packet_payload_floats("AFTER_RETURN", pkt_rx);

        SPP_OSAL_TaskDelay(BMP_TASK_DELAY_MS);
    }
}

retval_t BMP_ServiceInit(void* p_spi_bmp)
{
    if (p_spi_bmp == NULL) return SPP_ERROR_NULL_POINTER;

    s_spi = p_spi_bmp;

    // Setup ISR context
    s_bmp.int_pin       = (spp_uint32_t)INT_GPIO;
    s_bmp.int_intr_type = 1;
    s_bmp.int_pull      = 0;
    BmpInit(&s_bmp);

    // BMP config
    retval_t ret = bmp390_aux_config(s_spi);
    if (ret != SPP_OK) return ret;

    ret = bmp390_prepare_measure(s_spi);
    if (ret != SPP_OK) return ret;

    ret = bmp390_int_enable_drdy(s_spi);
    if (ret != SPP_OK) return ret;

    return SPP_OK;
}

retval_t BMP_ServiceStart(void)
{
    void* storage = SPP_OSAL_GetTaskStorage();
    if (storage == NULL) {
        SPP_LOGE(TAG, "No task storage");
        return SPP_ERROR;
    }

    // Create the task in loop
    void* th = SPP_OSAL_TaskCreate(
        (void*)bmp_service_task,
        "bmp_service",
        4096,
        NULL,
        BMP_TASK_PRIO,
        storage
    );

    if (th == NULL) {
        SPP_LOGE(TAG, "TaskCreate failed");
        return SPP_ERROR;
    }

    SPP_LOGI(TAG, "Task created");
    return SPP_OK;
}