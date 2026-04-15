#include "dataloggerSD.h"

#include "spp/hal/storage.h"
#include "spp/services/log.h"
#include "spp/core/types.h"
#include <string.h>

static const char *TAG = "DATALOGGER";

retval_t DATALOGGER_Init(Datalogger_t *p_logger, void *p_storage_cfg, const char *p_file_path)
{
    retval_t ret;

    memset(p_logger, 0, sizeof(Datalogger_t));
    p_logger->p_storage_cfg = p_storage_cfg;

    ret = SPP_Hal_storageMount(p_storage_cfg);
    if (ret != SPP_OK)
    {
        SPP_LOGE(TAG, "Storage mount failed");
        return ret;
    }

    p_logger->p_file = fopen(p_file_path, "w");
    if (p_logger->p_file == NULL)
    {
        SPP_LOGE(TAG, "Failed to open file: %s", p_file_path);
        (void)SPP_Hal_storageUnmount(p_storage_cfg);
        return SPP_ERROR;
    }

    p_logger->is_initialized = 1U;
    p_logger->logged_packets = 0U;

    SPP_LOGI(TAG, "Init OK file=%s", p_file_path);
    return SPP_OK;
}

retval_t DATALOGGER_LogPacket(Datalogger_t *p_logger, const SPP_Packet_t *p_packet)
{
    if ((p_logger->is_initialized == 0U) || (p_logger->p_file == NULL))
    {
        return SPP_ERROR;
    }

    int logSD;
    logSD =
        fprintf(p_logger->p_file,
                "pkt=%lu ver=%u apid=0x%04X seq=%u len=%u ts=%lu drop=%u crc=%u payload_hex=",
                (unsigned long)p_logger->logged_packets, (unsigned)p_packet->primaryHeader.version,
                (unsigned)p_packet->primaryHeader.apid, (unsigned)p_packet->primaryHeader.seq,
                (unsigned)p_packet->primaryHeader.payloadLen,
                (unsigned long)p_packet->secondaryHeader.timestampMs,
                (unsigned)p_packet->secondaryHeader.dropCounter, (unsigned)p_packet->crc);

    if (logSD < 0)
    {
        SPP_LOGE(TAG, "fprintf header failed");
        return SPP_ERROR;
    }


    for (spp_uint16_t i = 0U; i < p_packet->primaryHeader.payloadLen; i++)
    {
        // Write payload in hexa.
        logSD = fprintf(p_logger->p_file, "%02X", (unsigned)p_packet->payload[i]);
        if (logSD < 0)
        {
            SPP_LOGE(TAG, "fprintf payload failed");
            return SPP_ERROR;
        }

        // Add a separator
        if (i + 1U < p_packet->primaryHeader.payloadLen)
        {
            logSD = fprintf(p_logger->p_file, " ");
        }
    }
    // New line after writing the packet
    logSD = fprintf(p_logger->p_file, "\n");

    p_logger->logged_packets++;

    return SPP_OK;
}

retval_t DATALOGGER_Flush(Datalogger_t *p_logger)
{
    if ((p_logger->is_initialized == 0U) || (p_logger->p_file == NULL))
    {
        return SPP_ERROR;
    }

    if (fflush(p_logger->p_file) != 0)
    {
        SPP_LOGE(TAG, "fflush failed");
        return SPP_ERROR;
    }

    SPP_LOGI(TAG, "fflush OK");
    return SPP_OK;
}

retval_t DATALOGGER_Deinit(Datalogger_t *p_logger)
{
    retval_t ret;

    if (p_logger == NULL)
    {
        return SPP_ERROR_NULL_POINTER;
    }

    if (p_logger->p_file != NULL)
    {
        if (fflush(p_logger->p_file) != 0)
        {
            SPP_LOGE(TAG, "fflush failed during deinit");
        }

        fclose(p_logger->p_file);
        p_logger->p_file = NULL;
    }

    if (p_logger->p_storage_cfg != NULL)
    {
        ret = SPP_Hal_storageUnmount(p_logger->p_storage_cfg);
        if (ret != SPP_OK)
        {
            SPP_LOGE(TAG, "Storage unmount failed");
            p_logger->is_initialized = 0U;
            p_logger->p_storage_cfg = NULL;
            return ret;
        }
    }

    p_logger->is_initialized = 0U;
    p_logger->p_storage_cfg = NULL;

    SPP_LOGI(TAG, "Deinit OK");
    return SPP_OK;
}