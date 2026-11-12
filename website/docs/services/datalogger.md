# Datalogger

SD card packet logger implemented as an SPP consumer. It stores fixed-size 64-byte packets in a static buffer and writes them directly to SD card sectors through the storage HAL, without using a filesystem.

The datalogger subscribes to BMP390, ICM20948 and FSM telemetry packets at priority 0.

---

## Files

| File | Description |
|---|---|
| `datalogger.h` | Public API and buffer configuration |
| `datalogger.c` | Consumer implementation and sector-based logging |
| `storagePort.c` | ESP32-S3 SD card initialization and raw sector writing |

---

## API

```c
const SPP_SERVICE_ConsumerContract_t *SPP_SERVICES_DATALOGGER_getConsumerContract(void);
SPP_RetVal_t SPP_SERVICES_DATALOGGER_getInitStatus(void);
```

The datalogger implements three internal consumer callbacks:

- `init()`: Initializes SD card storage.
- `deliverToMailbox()`: Copies received packets into the static buffer.
- `consumeData()`: Writes the buffer to the SD card when full.


---

## Binary storage format

Packets are stored in their native 64-byte SPP format, without text conversion or filesystem headers.

| Parameter | Value |
|---|---|
| Packet size | 64 bytes |
| Buffer size | 1024 bytes |
| Buffer capacity | 16 packets |
| Sector size | 512 bytes |
| Sectors per write | 2 |
| First data sector | 16384 |

Once 16 packets have accumulated, the datalogger attempts to write two consecutive sectors. On success, the sector index advances and the packet buffer is cleared.

On write failure, the current implementation retains the buffer for a later attempt, but does not report the failure through `consumeData()`.

---

## Registration

The datalogger is registered as a PUBSUB consumer during application initialization in `main/custom.c`.

```c
const SPP_SERVICE_ConsumerContract_t *p_sdContract =
    SPP_SERVICES_DATALOGGER_getConsumerContract();

SPP_SERVICES_PUBSUB_registerConsumer(
    p_sdContract,
    sdSubscription
);
```

The subscription includes BMP390, ICM20948 and FSM telemetry.

## Hardware interface

SD card access is implemented through the storage HAL:

- `SPP_HAL_STORAGE_init()`: Initializes the SD card.
- `SPP_HAL_STORAGE_write()`: Writes consecutive 512-byte sectors.

The ESP32-S3 implementation uses SDSPI through the ESP-IDF SDMMC driver.

Raw sector writing bypasses the filesystem. The current implementation does not provide file creation, mounting, flushing or filesystem management.