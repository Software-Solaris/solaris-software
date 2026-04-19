# Solaris Software — CLAUDE.md

## Project Overview

**Solaris** is an ESP32-S3 embedded firmware project for environmental sensing (pressure, IMU, altitude). It is built on top of a custom lightweight protocol stack called **SPP (Solaris Packet Protocol)**. `solaris-v1` is the current active version.

Target: ESP32-S3 via ESP-IDF (managed by the devcontainer toolchain).
Build system: CMake (ESP-IDF component model).
Runtime model: **bare-metal superloop** — no RTOS, no tasks, no queues.

---

## Developer Tooling

### Terminal commands (devcontainer)

| Command | Description |
|---------|-------------|
| `help` | Show all available commands |
| `build` / `flash` / `monitor` / `fullflash` | ESP-IDF shortcuts |
| `goto <dest>` | Navigate to key directories (`spp`, `ports`, `services`, `compiler`, `tests`, `docs`) |
| `run_tests <path>` | cmake configure + build + ctest (Cgreen unit tests) |
| `template [type]` | Print Doxygen templates. Types: `h`, `c`, `fn`, `struct`, `enum`, `macro`. No argument = all. |

### Claude Code skills

| Command | Description |
|---------|-------------|
| `/edit <file>` | Apply Doxygen formatting, naming convention checks, and section dividers to a `.c` or `.h` file. Changes are applied directly. |

---

## Repository Layout

```
solaris-software/
├── solaris-v1/                  # Active firmware version
│   ├── main/                    # Application entry point
│   └── spp/                     # SPP library (self-contained, no ESP-IDF dependency in core)
│       ├── core/                # Packet format, types, return codes, core init
│       ├── hal/                 # HAL contract (headers + dispatch.c)
│       ├── services/            # Packet lifecycle + sensor services
│       │   ├── databank/        # Static packet pool
│       │   ├── pubsub/          # Synchronous publish-subscribe router
│       │   ├── log/             # Level-filtered logging
│       │   ├── bmp390/          # BMP390 pressure/altitude service
│       │   ├── icm20948/        # ICM20948 IMU service
│       │   └── datalogger/      # SD card packet logger
│       ├── util/                # CRC-16, macros, structof
│       ├── ports/               # Concrete HAL implementations
│       │   └── hal/
│       │       ├── esp32/       # ESP32-S3 polling SPI, GPIO ISR, SD card
│       │       └── stub/        # No-op stubs for host unit tests
│       ├── tests/               # Cgreen unit tests (run on host)
│       └── examples/            # baremetal_main.c — reference superloop
└── compiler/                    # ESP-IDF component wrappers
    ├── spp/                     # Wraps spp core + services as ESP-IDF component
    └── spp_ports/               # Wraps spp ports as ESP-IDF component
```

---

## SPP Architecture

There is **no OSAL layer**. SPP runs in a bare-metal superloop:

```
  ┌──────────────────────────────────────────────────────────┐
  │                      Application                         │
  │         superloop / bare-metal main / user code          │
  ├──────────────────────────────────────────────────────────┤
  │                  Sensor Services                         │
  │        bmp390 · icm20948 · datalogger                    │
  ├──────────────────────────────────────────────────────────┤
  │                  SPP Services                            │
  │      databank · pubsub · log · service registry          │
  ├──────────────────────────────────────────────────────────┤
  │                       HAL                                │
  │           SPI · GPIO · Storage · Time                    │
  │               (contract only)                            │
  ├──────────────────────────────────────────────────────────┤
  │                  Platform Ports                          │
  │   ports/hal/esp32/        ports/hal/stub/                │
  └──────────────────────────────────────────────────────────┘
```

### Data flow (bare-metal superloop pattern)

```
ISR sets volatile drdyFlag
  → superloop detects flag
    → ServiceTask()
        → SPP_Databank_getPacket()
        → SPP_Databank_packetData(pkt, apid, seq, data, len)
              fills all headers, copies payload, computes CRC-16
        → SPP_PubSub_publish(pkt)
              dispatches to all matching subscribers synchronously
              then calls SPP_Databank_returnPacket(pkt) automatically
```

### Pub/sub (synchronous, no queuing)

- `SPP_PubSub_subscribe(apid, handler, p_ctx)` — register a consumer
- `SPP_PubSub_publish(p_packet)` — dispatch + auto-return packet
- `K_SPP_APID_ALL (0xFFFFU)` — wildcard: subscribe to every packet
- `K_SPP_APID_LOG (0x0001U)` — reserved for SPP log message packets
- Max subscribers: `K_SPP_PUBSUB_MAX_SUBSCRIBERS` (default 8, in `util/macros.h`)

### Log → pub/sub bridge

`logPubSubOutput()` in `examples/baremetal_main.c` registers as the log output callback. Every `SPP_LOG*` call formats a `K_SPP_APID_LOG` packet and publishes it, so the SD card subscriber captures log messages alongside sensor data. A `s_logBusy` reentrancy guard prevents infinite loops.

---

## SPP core/

**`core/types.h`** — Portable types: `spp_uint8_t … spp_uint64_t`, `spp_bool_t`, `SPP_SpiInitCfg_t`, `SPP_StorageInitCfg_t`

**`core/returntypes.h`** — `SPP_RetVal_t`:
```c
K_SPP_OK, K_SPP_ERROR, K_SPP_NOT_ENOUGH_PACKETS, K_SPP_NULL_PACKET,
K_SPP_ERROR_ALREADY_INITIALIZED, K_SPP_ERROR_NULL_POINTER,
K_SPP_ERROR_NOT_INITIALIZED, K_SPP_ERROR_INVALID_PARAMETER,
K_SPP_ERROR_ON_SPI_TRANSACTION, K_SPP_ERROR_TIMEOUT,
K_SPP_ERROR_NO_PORT, K_SPP_ERROR_REGISTRY_FULL
```

**`core/packet.h`** — `SPP_Packet_t`:
```c
typedef struct { spp_uint8_t version; spp_uint16_t apid; spp_uint16_t seq;
                 spp_uint16_t payloadLen; } SPP_PacketPrimary_t;
typedef struct { spp_uint32_t timestampMs; spp_uint8_t dropCounter; } SPP_PacketSecondary_t;
typedef struct {
    SPP_PacketPrimary_t   primaryHeader;
    SPP_PacketSecondary_t secondaryHeader;
    spp_uint8_t           payload[K_SPP_PKT_PAYLOAD_MAX]; // 48 bytes
    spp_uint16_t          crc;
} SPP_Packet_t;
```
Constants: `K_SPP_PKT_VERSION = 1`, `K_SPP_PKT_PAYLOAD_MAX = 48`, `K_SPP_APID_LOG = 0x0001U`

**`core/core.h`**: `SPP_Core_setHalPort()`, `SPP_Core_init()` (calls `SPP_Databank_init` + `SPP_PubSub_init` internally)

---

## SPP services/

### databank/
Static pool of `K_SPP_DATABANK_SIZE` (default 5) packets.
- `SPP_Databank_getPacket()` → leases a packet (NULL if pool empty)
- `SPP_Databank_returnPacket()` → returns packet (called automatically by publish)
- `SPP_Databank_packetData(p_pkt, apid, seq, p_data, dataLen)`:
  - `memset` full struct to 0 (deterministic CRC over padding)
  - fills `primaryHeader`, `secondaryHeader.timestampMs = SPP_HAL_getTimeMs()`
  - `memcpy` payload, then computes CRC-16/CCITT over `offsetof(SPP_Packet_t, crc)` bytes

### pubsub/
Synchronous router. Max `K_SPP_PUBSUB_MAX_SUBSCRIBERS` (default 8) entries.
- `SPP_PubSub_subscribe(apid, handler, p_ctx)`
- `SPP_PubSub_publish(p_pkt)` — iterates subscribers, calls matching ones, returns packet to databank

### log/
`SPP_LOGE/W/I/D/V(tag, fmt, ...)` macros. Custom output registered with `SPP_Log_registerOutput()`.

### bmp390/
APID `0x0101`. ISR sets `bmpData.drdyFlag`. `BMP390_ServiceTask()` reads altitude/pressure/temp, publishes 3×float payload (12 bytes).

### icm20948/
APID `0x0201`. ISR sets `icmData.drdyFlag`. `ICM20948_ServiceTask()` drains DMP FIFO, publishes ax/ay/az/gx/gy/gz/mx/my/mz (9×float = 36 bytes).

### datalogger/
Pub/sub subscriber. `DATALOGGER_logPacket()` writes one line per packet:
- `K_SPP_APID_LOG` packets: payload string + `\n`
- Sensor packets: `ts=<ms> apid=0x<X> seq=<N> len=<N> payload_hex=...\n`
Flush every `K_SD_FLUSH_EVERY` packets to limit data loss on power cut.

---

## HAL contract (SPP_HalPort_t)

```c
typedef struct {
    SPP_RetVal_t  (*spiBusInit)(void);
    void         *(*spiGetHandle)(spp_uint8_t deviceIdx);
    SPP_RetVal_t  (*spiDeviceInit)(void *p_handle);
    SPP_RetVal_t  (*spiTransmit)(void *p_handle, spp_uint8_t *p_data, spp_uint8_t len);
    SPP_RetVal_t  (*gpioConfigInterrupt)(spp_uint32_t pin, spp_uint32_t intrType, spp_uint32_t pull);
    SPP_RetVal_t  (*gpioRegisterIsr)(spp_uint32_t pin, void *p_isrCtx);
    SPP_RetVal_t  (*storageMount)(void *p_cfg);    // optional (NULL if no storage)
    SPP_RetVal_t  (*storageUnmount)(void *p_cfg);  // optional
    spp_uint32_t  (*getTimeMs)(void);
    void          (*delayMs)(spp_uint32_t ms);
} SPP_HalPort_t;
```

GPIO ISR context:
```c
typedef struct { volatile spp_bool_t *p_flag; } SPP_GpioIsrCtx_t;
// ISR: *p_ctx->p_flag = true;  (no RTOS calls, no yield)
```

---

## ESP32 HAL port (ports/hal/esp32/hal_esp32.c)

Exports `const SPP_HalPort_t g_esp32HalPort`. All internal functions named `HAL_ESP32S3_functionName()`.

- Polling SPI (`spi_device_polling_transmit`) — no FreeRTOS semaphore
- Time via `esp_timer_get_time()` — no `xTaskGetTickCount()`
- GPIO ISR does not call `portYIELD_FROM_ISR()`

SPI device index mapping (in `macros_esp32.h`):
- Index 0: ICM20948 (CS GPIO 21, 1 MHz, MODE0)
- Index 1: BMP390 (CS GPIO 18, 500 kHz, MODE0)
- SD card: GPIO 8 (via storageMount, not SPI device init)

Pin map: MISO=47, MOSI=38, CLK=48

---

## Startup sequence

```c
// 1. Register HAL port
SPP_Core_setHalPort(&g_esp32HalPort);

// 2. Init core (Databank + PubSub init called internally)
SPP_Core_init();

// 3. Register log → pub/sub bridge before any SPP_LOG* calls
SPP_Log_registerOutput(logPubSubOutput);

// 4. Init SD card logger
DATALOGGER_init(&s_logger, &s_storageCfg, "/sdcard/log.txt");

// 5. Subscribe consumers
SPP_PubSub_subscribe(K_SPP_APID_ALL, sdLogHandler, &s_logger);

// 6. Init SPI bus and devices
SPP_HAL_spiBusInit();
SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(0U));  // ICM20948
SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(1U));  // BMP390

// 7. Register, init and start services
SPP_Service_register(&g_icm20948ServiceDesc, &s_icmCtx, &s_icmCfg);
SPP_Service_register(&g_bmp390ServiceDesc,   &s_bmpCtx, &s_bmpCfg);
SPP_Service_initAll();
SPP_Service_startAll();

// 8. Superloop
for (;;) {
    if (s_bmpCtx.bmpData.drdyFlag)  BMP390_ServiceTask(&s_bmpCtx);
    if (s_icmCtx.icmData.drdyFlag)  ICM20948_ServiceTask(&s_icmCtx);
}
```

---

## Service descriptor

```c
typedef struct {
    const char   *p_name;
    spp_uint16_t  apid;
    size_t        ctxSize;
    SPP_RetVal_t (*init)  (void *p_ctx, const void *p_cfg);
    SPP_RetVal_t (*start) (void *p_ctx);
    SPP_RetVal_t (*stop)  (void *p_ctx);
    SPP_RetVal_t (*deinit)(void *p_ctx);
} SPP_ServiceDesc_t;
```

---

## APID allocation

| Range | Owner |
|---|---|
| `0x0001` | `K_SPP_APID_LOG` — log message packets |
| `0x0100`–`0x01FF` | Solaris sensor services (BMP390 = `0x0101`) |
| `0x0200`–`0x02FF` | Reserved (ICM20948 = `0x0201`) |
| `0x0300`–`0xFFFE` | User-defined |
| `0xFFFF` | `K_SPP_APID_ALL` — pub/sub wildcard |

---

## CRC-16/CCITT

Polynomial 0x1021, init 0xFFFF. Computed by `SPP_Databank_packetData()` over the full packet excluding the `crc` field (`offsetof(SPP_Packet_t, crc)` bytes). The packet is `memset` to 0 first so padding bytes are always 0, making the CRC deterministic.

---

## util/macros.h — Compile-time constants

| Constant | Default | Purpose |
|---|---|---|
| `K_SPP_DATABANK_SIZE` | 5 | Packet pool size |
| `K_SPP_PUBSUB_MAX_SUBSCRIBERS` | 8 | Max pub/sub subscribers |
| `K_SPP_MAX_SERVICES` | 16 | Service registry slots |

---

## Coding Conventions

### Naming rules

| Element | Convention | Example |
|---------|-----------|---------|
| Constants / `#define` | `K_MODULE_NAME` | `K_SPP_PKT_PAYLOAD_MAX` |
| Enum values | `K_MODULE_ENUM_VALUE` | `K_SPP_LOG_VERBOSE` |
| Return-code values | `K_SPP_OK`, `K_SPP_ERROR` | (shared cross-module, no prefix rule) |
| Portable base types | lowercase snake | `spp_uint8_t`, `spp_bool_t` |
| Struct / Union / Enum types | `MODULE_TypeName_t` | `SPP_Packet_t`, `ICM20948_Data_t` |
| Public functions | `MODULE_functionName()` camelCase | `SPP_Databank_init()`, `ICM20948_ServiceTask()` |
| HAL port internal functions | `HAL_TARGET_functionName()` | `HAL_ESP32S3_spiBusInit()` |
| Pointer parameters | `p_name` prefix | `p_data`, `p_cfg` |
| Static module-level vars | `s_name` prefix | `s_initialized`, `s_logBusy` |
| Static file-level constants | `k_name` prefix | `k_tag` |

### Doxygen style (mandatory on all public symbols)

```c
/**
 * @file module.h
 * @brief One-line description.
 *
 * Naming conventions used in this file:
 * - Global constants/macros: K_SPP_MODULE_*
 * - Types: SPP_Module*_t
 * - Public functions: SPP_Module_functionName()
 */

/**
 * @brief Short description.
 * @param[in]  p_cfg  Pointer to configuration struct.
 * @return K_SPP_OK on success, error code otherwise.
 */
```

### Section dividers (mandatory in .c/.h files)

```c
/* ----------------------------------------------------------------
 * Section Name
 * ---------------------------------------------------------------- */
```

---

## Reference: lley-core (Lely Industries)

Location: `/home/user/Documents/lley-core`
Purpose: Industrial C/C++ library for CANopen. Studied as architectural reference for SPP.

Key patterns adopted:
- OOP-in-C via function-pointer structs (vtables) → `SPP_HalPort_t`
- `container_of` / `structof` → `SPP_STRUCTOF()` in `util/structof.h`
- Static-first allocation (no malloc by default)
- Compile-time feature flags (`SPP_NO_MALLOC`, `SPP_NO_STORAGE`)
- Callback registration pattern → `SPP_Log_registerOutput()`

---

## Hardware Pin Map (ESP32-S3)

| Signal | GPIO |
|--------|------|
| SPI MISO | 47 |
| SPI MOSI | 38 |
| SPI CLK | 48 |
| CS ICM20948 | 21 |
| CS BMP390 | 18 |
| CS SD card | 8 |
| BMP390 DRDY | 5 |
| ICM20948 INT | 4 |
