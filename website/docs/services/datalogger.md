# Datalogger

SD card packet logger. Mounts a FAT filesystem over SPI, opens a log file, and writes packets as structured text — one line per packet. Subscribes to `K_SPP_APID_ALL` at `PRIO_LOW` via the module descriptor, so every published packet is appended to the log file automatically.

---

## Files

| File | Description |
|---|---|
| `datalogger.h` | Public API and `Datalogger_t` context struct |
| `datalogger.c` | Implementation — mount, open, write, flush, close |

---

## Key types

```c
typedef struct {
    /* Config — set at declaration */
    void       *p_storageCfg;  // Pointer to SPP_StorageInitCfg_t
    const char *p_filePath;    // Absolute path of the file to create/overwrite

    /* Runtime — filled by init, do not set manually */
    FILE       *p_file;
    spp_bool_t  is_open;
    uint32_t    logged_packets;
} Datalogger_t;
```

---

## API

```c
SPP_RetVal_t SPP_SERVICES_DATALOGGER_init(Datalogger_t *p_logger);
SPP_RetVal_t SPP_SERVICES_DATALOGGER_logPacket(Datalogger_t *p_logger, const SPP_Packet_t *p_packet);
SPP_RetVal_t SPP_SERVICES_DATALOGGER_flush(Datalogger_t *p_logger);
SPP_RetVal_t SPP_SERVICES_DATALOGGER_deinit(Datalogger_t *p_logger);
```

---

## Log line format

**Log message packets** (`apid == K_SPP_APID_LOG`):
```
[I] BMP390: altitude = 1234.56 m
```

**Sensor/telemetry packets** (all other APIDs):
```
ts=12345 apid=0x0004 seq=7 len=12 payload_hex=44 9A 4B 45 00 00 B8 43 00 80 FF 42
```

Each line is terminated with `\n`. The logger flushes automatically every 20 packets.

---

## Usage via module descriptor

```c
#include "spp/services/datalogger/datalogger.h"

static const SPP_StorageInitCfg_t s_storageCfg = {
    .p_basePath          = "/sdcard",
    .spiHostId           = 1,
    .pinCs               = 9,
    .maxFiles            = 5U,
    .allocationUnitSize  = 16384U,
    .formatIfMountFailed = false,
};

static Datalogger_t s_logger = {
    .p_storageCfg = (void *)&s_storageCfg,
    .p_filePath   = "/sdcard/log.txt",
};

// Register before the superloop — subscribes to K_SPP_APID_ALL at PRIO_LOW automatically
SPP_SERVICES_register(&g_sdLoggerModule, &s_logger);
```

---

## Hardware configuration (ESP32-S3)

- SPI bus: SPI2_HOST (host ID 1)
- CS GPIO: 8 or 9
- Mount point: `/sdcard`
- Default allocation unit: 16 KB
