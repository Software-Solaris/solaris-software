#include "core/core.h"
#include "core/returntypes.h"

#include "spi.h"
#include "bmp390.h"
#include "gpio_int.h"

#include "services/databank/databank.h"
#include "services/db_flow/db_flow.h"

#include "osal/eventgroups.h"
#include "osal/task.h"
#include "spp_log.h"

#include <string.h>
#include <math.h>

static const char* TAG = "BMP_PKT_FLOW";

static bmp_data_t s_bmp;
static void*      s_spi_bmp = NULL;

#define APID_BMP_DBG  0x0101
#define DEBUG_EVERY_N 100u

typedef struct __attribute__((packed)) {
    float alt_m;
    float p_pa;
    float t_c;
} bmp_dbg_payload_t;

static spp_uint32_t SPP_OSAL_GetTimeMs_Fallback(void)
{
    return 0;
}

void app_main(void)
{
    retval_t ret;

    Core_Init();
    SPP_LOGI(TAG, "Starting BMP390 -> DataBank -> DB_FLOW (loop)");

    ret = SPP_DATABANK_init();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPP_DATABANK_init failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    ret = DB_FLOW_Init();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "DB_FLOW_Init failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    ret = SPP_HAL_SPI_BusInit();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPP_HAL_SPI_BusInit failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    void *p_spi_icm_dummy = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_spi_icm_dummy);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPP_HAL_SPI_DeviceInit(ICM dummy) failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    s_spi_bmp = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(s_spi_bmp);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPP_HAL_SPI_DeviceInit(BMP) failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    ret = bmp390_aux_config(s_spi_bmp);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "bmp390_aux_config failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    s_bmp.int_pin       = (spp_uint32_t)INT_GPIO;
    s_bmp.int_intr_type = 1;
    s_bmp.int_pull      = 0;
    BmpInit(&s_bmp);

    ret = bmp390_prepare_measure(s_spi_bmp);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "bmp390_prepare_measure failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    ret = bmp390_int_enable_drdy(s_spi_bmp);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "bmp390_int_enable_drdy failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    bmp390_temp_params_t temp_params;
    bmp390_press_params_t press_params;

    ret = bmp390_calibrate_temp_params(s_spi_bmp, &temp_params);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "bmp390_calibrate_temp_params failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    ret = bmp390_calibrate_press_params(s_spi_bmp, &press_params);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "bmp390_calibrate_press_params failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    spp_uint16_t seq = 0;
    spp_uint8_t  drop = 0;
    spp_uint32_t produced = 0;
    spp_uint32_t consumed = 0;

    for (;;)
    {
        osal_eventbits_t bits = 0;
        ret = OSAL_EventGroupWaitBits(
            s_bmp.p_event_group,
            BMP390_EVT_DRDY,
            1,
            0,
            1000,
            &bits
        );

        if (!((ret == SPP_OK) && ((bits & BMP390_EVT_DRDY) != 0u))) {
            continue;
        }

        spp_uint32_t raw_temp = 0;
        spp_uint32_t raw_press = 0;

        ret = bmp390_read_raw_temp(s_spi_bmp, &raw_temp);
        if (ret != SPP_OK) { drop++; continue; }

        ret = bmp390_read_raw_press(s_spi_bmp, &raw_press);
        if (ret != SPP_OK) { drop++; continue; }

        float t_c = bmp390_compensate_temperature(raw_temp, &temp_params);
        float p_pa = bmp390_compensate_pressure(raw_press, t_c, &press_params);
        float alt_m = 44330.0f * (1.0f - powf(p_pa / 101325.0f, 1.0f / 5.255f));

        spp_packet_t* pkt = SPP_DATABANK_getPacket();
        if (pkt == NULL) {
            drop++;
            continue;
        }

        pkt->primary_header.version = SPP_PKT_VERSION;
        pkt->primary_header.apid    = APID_BMP_DBG;
        pkt->primary_header.seq     = seq++;
        pkt->primary_header.payload_len = (spp_uint16_t)sizeof(bmp_dbg_payload_t);

        pkt->secondary_header.timestamp_ms = SPP_OSAL_GetTimeMs_Fallback();
        pkt->secondary_header.drop_counter = drop;
        pkt->crc = 0;

        bmp_dbg_payload_t pl = { alt_m, p_pa, t_c };
        memset(pkt->payload, 0, SPP_PKT_PAYLOAD_MAX);
        memcpy(pkt->payload, &pl, sizeof(pl));

        ret = DB_FLOW_PushReady(pkt);
        if (ret != SPP_OK) {
            drop++;
            (void)SPP_DATABANK_returnPacket(pkt);
            continue;
        }

        produced++;

        // Always consume and return packets to avoid pool exhaustion
        for (;;)
        {
            spp_packet_t* rdy = NULL;
            if (DB_FLOW_PopReady(&rdy) != SPP_OK || rdy == NULL) {
                break;
            }

            consumed++;

            if ((consumed % DEBUG_EVERY_N) == 0u) {
                bmp_dbg_payload_t dbg;
                memcpy(&dbg, rdy->payload, sizeof(dbg));
                SPP_LOGI(TAG, "OK seq=%u alt=%.2f p=%.2f t=%.2f prod=%lu cons=%lu ready=%lu",
                         (unsigned)rdy->primary_header.seq,
                         dbg.alt_m, dbg.p_pa, dbg.t_c,
                         (unsigned long)produced,
                         (unsigned long)consumed,
                         (unsigned long)DB_FLOW_ReadyCount());
            }

            (void)SPP_DATABANK_returnPacket(rdy);
        }
    }
}