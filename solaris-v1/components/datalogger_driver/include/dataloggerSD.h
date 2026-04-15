#ifndef DATALOGGER_H
#define DATALOGGER_H

#include "spp/core/returntypes.h"
#include "spp/core/packet.h"
#include <stdio.h>
#include <stdint.h>

typedef struct
{
    void *p_storage_cfg;
    FILE *p_file;
    uint8_t is_initialized;
    uint32_t logged_packets;
} Datalogger_t;

retval_t DATALOGGER_Init(Datalogger_t *p_logger, void *p_storage_cfg, const char *p_file_path);

retval_t DATALOGGER_LogPacket(Datalogger_t *p_logger, const SPP_Packet_t *p_packet);

retval_t DATALOGGER_Flush(Datalogger_t *p_logger);

retval_t DATALOGGER_Deinit(Datalogger_t *p_logger);

#endif /* DATALOGGER_H */