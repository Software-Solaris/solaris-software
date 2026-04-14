# Software Architecture

This document describes the repository structure, the firmware layers, and how the components fit together in `solaris-v1`.

---

## Repository structure

```
solaris-software/
├── solaris-v1/                  # Active firmware version
│   ├── main/                    # Application entry point
│   │   ├── main.c               # SPI init → ICM DMP init → FIFO polling loop
│   │   ├── bmpService.c/h       # BMP390 service (refactored, PascalCase)
│   │   └── bmp_service.c/h      # BMP390 service (original, snake_case)
│   ├── components/
│   │   ├── icm_driver/          # ICM20948 IMU: DMP firmware, 9-axis output
│   │   ├── pressureSensorDriver/# BMP390 barometer: calibration, altitude
│   │   ├── general/             # Common macros, data types, GPIO interrupts
│   │   ├── datalogger_driver/   # SD card datalogger (in progress)
│   │   ├── spp_wrapper/         # CMake wrapper for the SPP core library
│   │   └── spp_port_wrapper/    # CMake wrapper for the SPP ESP32 port
│   └── external/                # Git submodules
│       ├── spp/                 # Solaris Packet Protocol library
│       └── spp-ports/           # ESP32 + FreeRTOS HAL/OSAL implementations
├── solaris-v0.1/                # Legacy: BMP390 standalone (ESP-IDF native)
├── solaris-v0.2/                # Legacy: ICM20948 standalone (ESP-IDF native)
├── website/                     # Project website source
├── .devcontainer/               # Docker build environment for VS Code
└── .vscode/                     # VS Code tasks and debug config
```

---

## Firmware layers

Solaris v1 is structured in four layers. Each layer only depends on the one below it — no layer calls upward.

```
┌─────────────────────────────────────────────────────┐
│                   Application                       │
│              main.c  ·  bmpService.c                │
├──────────────────────┬──────────────────────────────┤
│   Sensor Drivers     │       SPP Services           │
│  icm_driver          │  Databank · DB Flow          │
│  pressureSensorDriver│  Logging  · Checksum         │
├──────────────────────┴──────────────────────────────┤
│                  SPP Core                           │
│        packet · types · returntypes                 │
├─────────────────────────────────────────────────────┤
│          HAL                  │       OSAL          │
│  SPI · GPIO · Storage         │  Task · Queue       │
│  (ESP32 implementation)       │  EventGroup · Mutex │
│  in spp-ports/hal/esp32/      │  in spp-ports/osal/ │
│                               │  freertos/          │
└───────────────────────────────┴─────────────────────┘
         Hardware: ESP32-S3 · SPI bus · FreeRTOS
```

### HAL — Hardware Abstraction Layer

Wraps all hardware access. The SPP core defines the interfaces (`spp/hal/`); the implementations live in `spp-ports/hal/esp32/`.

| Interface | What it does |
|---|---|
| `SPP_HAL_SPI_BusInit()` | Initialises SPI2_HOST (MISO 47, MOSI 38, CLK 48) |
| `SPP_HAL_SPI_DeviceInit()` | Registers a device on the bus (ICM at CS 21, BMP at CS 18) |
| `SPP_HAL_SPI_Transmit()` | Full-duplex transaction; handles R/W bit and dummy-byte offsets per device |
| `SPP_HAL_GPIO_ConfigInterrupt()` | Wraps `gpio_config` for interrupt pins |
| `SPP_HAL_GPIO_RegisterISR()` | Installs per-pin ISR; signals an FreeRTOS event group from the ISR |
| `SPP_HAL_Storage_Mount/Unmount()` | SD card via FATFS + SDSPI |

### OSAL — Operating System Abstraction Layer

Wraps FreeRTOS primitives so the SPP core and drivers are portable. Defined in `spp/osal/`; implemented in `spp-ports/osal/freertos/`.

| Primitive | SPP wrapper |
|---|---|
| Tasks | `SPP_OSAL_TaskCreate()` — static allocation, 4096-byte stack pool |
| Event groups | `SPP_OSAL_EventGroupCreate()` — static pool of 5 groups |
| Queues | `SPP_OSAL_QueueCreate/Send/Receive()` |
| Mutexes | `OSAL_MutexCreate/Take/Give()` |
| Semaphores | `OSAL_SemaphoreCreate/Take/Give()` |

### SPP Core

The Solaris Packet Protocol defines a common packet format for all sensor data:

```
spp_packet_t
├── Primary header    version · APID · sequence · payload_len
├── Secondary header  timestamp_ms · drop_counter
├── Payload           up to 48 bytes of sensor data
└── CRC16
```

Key services built on top:

- **Databank**: static pool of 5 packets. Drivers call `SPP_DATABANK_getPacket()`, fill it, and return it via `SPP_DATABANK_returnPacket()`.
- **DB Flow**: 16-slot FIFO of packet pointers. Producers push with `DB_FLOW_PushReady()`; consumers pop with `DB_FLOW_PopReady()`.
- **Logging**: `SPP_LOGE/W/I/D/V(tag, fmt, ...)` — level-filtered, output via registered callback.

### Sensor drivers

**ICM20948** (`components/icm_driver/`)
- 9-axis IMU: accelerometer (±4 g), gyroscope (±2000 °/s), magnetometer AK09916
- Runs the DMP (Digital Motion Processor): firmware loaded on every boot, computes 9-axis quaternions on-chip
- Outputs 42-byte FIFO packets at 225 Hz
- Full init sequence in `ICM20948_configDmpInit()`; FIFO polling in `ICM20948_checkFifoData()`

**BMP390** (`components/pressureSensorDriver/`)
- Barometer: pressure, temperature, altitude
- Altitude formula: `44330 × (1 − (P / 101325)^(1/5.255))`
- Data-ready interrupt via GPIO 5 → FreeRTOS event group
- Normal mode, 50 Hz ODR, IIR coefficient 1

---

## Naming conventions

| Pattern | Used for |
|---|---|
| `K_ICM20948_*`, `K_BMP_*` | Constants and macros |
| `ICM20948_*_t`, `BMP390_*_t` | Type definitions |
| `ICM20948_*()`, `BMP390_*()` | Public functions |
| `p_name` | Pointer parameters |
| `s_name` | Static variables |
| `RegXxx_t` | Register union types (`.value` raw byte / `.bits` named fields) |

---

## Return types

All functions return `retval_t`:

| Value | Meaning |
|---|---|
| `SPP_OK` | Success |
| `SPP_ERROR` | Generic error |
| `SPP_ERROR_NULL_POINTER` | Null argument passed |
| `SPP_ERROR_NOT_INITIALIZED` | Component not initialised |
| `SPP_ERROR_INVALID_PARAMETER` | Bad argument value |
| `SPP_ERROR_ON_SPI_TRANSACTION` | SPI bus error |
