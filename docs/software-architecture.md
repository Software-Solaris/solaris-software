# Software Architecture

This document describes the repository structure, the firmware layers, the build system, and how everything fits together in `solaris-v1`. It is written for students who know basic C but have not worked on embedded systems before.

---

## Repository structure

```
solaris-software/
├── solaris-v1/                  # Active firmware version
│   ├── main/                    # Application entry point (main.c)
│   ├── compiler/                # ESP-IDF components (CMake build adapters)
│   │   ├── spp/                 # Compiles SPP core + selected services
│   │   └── spp_ports/           # Compiles platform ports (OSAL + HAL)
│   └── spp/                     # SPP library (git submodule)
│       ├── include/spp/         # Public headers
│       ├── src/                 # Core implementations
│       ├── ports/               # Platform ports (freertos, baremetal, esp32)
│       ├── services/            # Sensor services (bmp390, icm20948, datalogger)
│       └── tests/               # Unit tests
├── archive/                     # Legacy firmware binaries (v0.1, v0.2)
├── docs/                        # Architecture documentation
├── .devcontainer/               # Docker build environment
├── .vscode/                     # VS Code tasks and debug config
└── .github/workflows/           # CI/CD pipeline
```

---

## Firmware layers

Solaris v1 is structured in four layers. Each layer only depends on the one below it — no layer calls upward.

```
  ┌──────────────────────────────────────────────────────┐
  │                    Application                       │
  │                  main/main.c                         │
  ├──────────────────────────────────────────────────────┤
  │                     Services                         │
  │   BMP390 · ICM20948 · Datalogger  (sensor services)  │
  │   Databank · DB Flow · Logging    (SPP services)     │
  ├──────────────────────────────────────────────────────┤
  │                     SPP Core                         │
  │           packet · types · returntypes               │
  ├────────────────────────┬─────────────────────────────┤
  │          HAL           │           OSAL              │
  │  SPI · GPIO · Storage  │  Tasks · Queues · Events    │
  │  (interface only —     │  (interface only —          │
  │   defined in           │   defined in                │
  │   spp/include/spp/hal/)│   spp/include/spp/osal/)    │
  ├────────────────────────┴─────────────────────────────┤
  │              Platform ports                          │
  │  spp/ports/hal/esp32/     spp/ports/osal/freertos/   │
  │  spp/ports/hal/esp32_bm/  spp/ports/osal/baremetal/  │
  └──────────────────────────────────────────────────────┘
           Hardware: ESP32-S3 · SPI bus · FreeRTOS
```

### HAL — Hardware Abstraction Layer

HAL defines *what* hardware operations exist (SPI, GPIO, SD card storage) but contains no implementation. The headers in `spp/include/spp/hal/` describe the `SPP_HalPort_t` struct — a table of function pointers. Platform-specific code fills this struct in `spp/ports/hal/`.

| Function | What it does |
|---|---|
| `SPP_Hal_spiBusInit()` | Initialises SPI2_HOST (MISO 47, MOSI 38, CLK 48) |
| `SPP_Hal_spiDeviceInit()` | Registers a device on the bus (ICM at CS 21, BMP at CS 18) |
| `SPP_Hal_spiTransmit()` | Full-duplex SPI transaction; handles R/W bit per device |
| `SPP_Hal_gpioConfigInterrupt()` | Configures a GPIO pin for edge-triggered interrupts |
| `SPP_Hal_gpioRegisterIsr()` | Installs an ISR that signals a FreeRTOS event group |
| `SPP_Hal_storageMount()` | Mounts the SD card via FATFS + SDSPI |
| `SPP_Hal_storageUnmount()` | Unmounts the SD card safely |

### OSAL — Operating System Abstraction Layer

OSAL wraps OS primitives so all SPP code above it remains portable. The interface is defined in `spp/include/spp/osal/`; the FreeRTOS implementation lives in `spp/ports/osal/freertos/`.

| Function | What it does |
|---|---|
| `SPP_Osal_taskCreate()` | Creates a task (statically allocated, 4096-word stack pool) |
| `SPP_Osal_taskDelayMs()` | Delays the calling task by N milliseconds |
| `SPP_Osal_queueCreate()` | Creates a message queue |
| `SPP_Osal_queueSend()` | Sends an item to a queue (with timeout) |
| `SPP_Osal_queueRecv()` | Receives an item from a queue (with timeout) |
| `SPP_Osal_mutexCreate()` | Creates a mutex |
| `SPP_Osal_mutexLock()` | Locks a mutex (with timeout) |
| `SPP_Osal_mutexUnlock()` | Unlocks a mutex |
| `SPP_Osal_eventCreate()` | Creates an event group |
| `SPP_Osal_eventWait()` | Blocks until specific event bits are set (with timeout) |
| `SPP_Osal_eventSetFromIsr()` | Sets event bits from an ISR context |

### SPP Core

The Solaris Packet Protocol defines a common packet format for all sensor data:

```
SPP_Packet_t
├── Primary header    APID · sequence · payload_len
├── Secondary header  timestamp_ms · drop_counter
├── Payload           up to 48 bytes of sensor data
└── CRC16
```

Built-in services on top of Core:

- **Databank**: static pool of 5 packets. Call `SPP_Databank_getPacket()` to lease one, fill it, then return it via `SPP_Databank_returnPacket()`. No `malloc` is ever called.
- **DB Flow**: 16-slot circular FIFO of packet pointers. Sensor tasks push with `SPP_DbFlow_pushReady()`; the logger task pops with `SPP_DbFlow_popReady()`.
- **Logging**: `SPP_LOGE/W/I/D/V(tag, fmt, ...)` — level-filtered, output via registered callback.

### Sensor services

**BMP390** (`spp/services/bmp390/`)
- Barometer: pressure, temperature, altitude
- Altitude formula: `44330 × (1 − (P / 101325)^(1/5.255))`
- Data-ready interrupt via GPIO → FreeRTOS event group
- Normal mode, 50 Hz ODR, IIR filter coefficient 1

**ICM20948** (`spp/services/icm20948/`)
- 9-axis IMU: accelerometer (±4 g), gyroscope (±2000 °/s), magnetometer AK09916
- DMP (Digital Motion Processor) computes 9-axis quaternions on-chip
- Outputs 42-byte FIFO packets at 225 Hz

**Datalogger** (`spp/services/datalogger/`)
- Writes SPP packets to a file on the SD card
- Configured via `SPP_StorageInitCfg_t` (mount path, CS pin, file count limit)

---

## Build system

### How ESP-IDF finds the SPP library

ESP-IDF discovers components by scanning directories listed in `EXTRA_COMPONENT_DIRS`. In `solaris-v1/CMakeLists.txt`, this is set to the `compiler/` directory:

```cmake
set(EXTRA_COMPONENT_DIRS compiler)
```

ESP-IDF then finds two components inside `compiler/`:

- `compiler/spp/` — compiles the SPP core and any enabled services
- `compiler/spp_ports/` — compiles the enabled OSAL and HAL ports

The actual SPP source code lives in `solaris-v1/spp/` as a **git submodule** — a separate repository embedded inside this one. After cloning `solaris-software`, you must initialise it:

```bash
git submodule update --init
```

### CMake options

Both `compiler/spp/CMakeLists.txt` and `compiler/spp_ports/CMakeLists.txt` expose CMake options to include or exclude individual services and ports. All options default to ON.

| Option | Default | What it enables |
|---|---|---|
| `SPP_SERVICE_BMP390` | ON | BMP390 pressure sensor service |
| `SPP_SERVICE_ICM20948` | ON | ICM20948 IMU service |
| `SPP_SERVICE_DATALOGGER` | ON | SD card datalogger service |
| `SPP_OSAL_FREERTOS` | ON | FreeRTOS OSAL port |
| `SPP_OSAL_BAREMETAL` | ON | Baremetal cooperative scheduler |
| `SPP_HAL_ESP32` | ON | ESP32 HAL (FreeRTOS SPI) |
| `SPP_HAL_ESP32_BM` | ON | ESP32 HAL (polling SPI, used with baremetal) |

To disable a service at build time, pass `-D<OPTION>=OFF` to `idf.py`:

```bash
idf.py build -DSPP_SERVICE_ICM20948=OFF -DSPP_OSAL_FREERTOS=OFF
```

---

## Adding a new ESP-IDF component

If you want to add a new component (for example, a new sensor driver) without modifying the SPP library itself, create it directly inside `compiler/`:

### Step 1 — Create the component directory and CMakeLists.txt

```
compiler/
└── my_component/
    ├── CMakeLists.txt
    ├── include/
    │   └── my_component.h
    └── src/
        └── my_component.c
```

### Step 2 — Write the minimal CMakeLists.txt

```cmake
idf_component_register(
    SRCS
        "src/my_component.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        spp          # if this component uses the SPP library
)
```

### Step 3 — Done

ESP-IDF picks up the new component automatically because `compiler/` is already listed in `EXTRA_COMPONENT_DIRS`. No changes to the top-level `CMakeLists.txt` are needed.

---

## HAL and OSAL contracts

The full port contracts — the structs you fill when porting to a new platform — are defined in `spp/include/spp/hal/port.h` and `spp/include/spp/osal/port.h`.

### SPP_HalPort_t

| Field | What it does |
|---|---|
| `spiBusInit` | Initialise the SPI bus |
| `spiGetHandle` | Return a device handle by index |
| `spiDeviceInit` | Register a device on the bus |
| `spiTransmit` | Perform a full-duplex SPI transaction |
| `gpioConfigInterrupt` | Configure a GPIO pin for interrupts |
| `gpioRegisterIsr` | Attach an ISR to a GPIO pin |
| `storageMount` | Mount a storage medium |
| `storageUnmount` | Unmount a storage medium |
| `getTimeMs` | Return the current system time in milliseconds |

### SPP_OsalPort_t

| Field | What it does |
|---|---|
| `taskCreate` | Create a task |
| `taskDelete` | Delete a task (NULL = self) |
| `taskDelayMs` | Delay the calling task |
| `getTickMs` | Return current tick in milliseconds |
| `queueCreate` | Create a message queue |
| `queueSend` | Send an item (with timeout) |
| `queueRecv` | Receive an item (with timeout) |
| `queueCount` | Number of items currently in the queue |
| `mutexCreate` | Create a mutex |
| `mutexLock` | Lock a mutex (with timeout) |
| `mutexUnlock` | Unlock a mutex |
| `eventCreate` | Create an event group |
| `eventWait` | Wait for event bits (with timeout) |
| `eventSetFromIsr` | Set event bits from an ISR |

---

## Return types and error handling

All SPP functions return `SPP_RetVal_t`. Check the return value of every call.

| Value | Meaning |
|---|---|
| `K_SPP_OK` | Success |
| `K_SPP_ERROR` | Generic error |
| `K_SPP_ERROR_NULL_POINTER` | A required pointer argument was NULL |
| `K_SPP_ERROR_NOT_INITIALIZED` | Component was not initialised before use |
| `K_SPP_ERROR_INVALID_PARAMETER` | An argument value is out of range or nonsensical |
| `K_SPP_ERROR_ON_SPI_TRANSACTION` | The SPI bus returned an error |
| `K_SPP_ERROR_TIMEOUT` | A blocking operation exceeded its timeout |
| `K_SPP_ERROR_NO_PORT` | A HAL or OSAL port was not registered before use |
| `K_SPP_ERROR_REGISTRY_FULL` | The service registry has no free slots |

For rich error context (source file and line number), use `SPP_ERR_getCtx()` which returns an `SPP_ErrCtx_t`. In implementations, use the macro `SPP_ERR_RETURN(K_SPP_ERROR_NULL_POINTER)` instead of a plain `return` — it records the call site before returning.

```c
if (p_cfg == NULL) {
    SPP_ERR_RETURN(K_SPP_ERROR_NULL_POINTER);
}
```

---

## Naming conventions

| Pattern | Used for | Example |
|---|---|---|
| `K_MODULE_*` | Constants and `#define` macros | `K_SPP_PKT_PAYLOAD_MAX`, `K_ICM20948_BANK0` |
| `K_MODULE_ENUM_VALUE` | Enum values | `K_SPP_LOG_VERBOSE`, `K_SPP_SPI_MODE0` |
| `SPP_Module*_t` | Type definitions (struct, enum, typedef) | `SPP_Packet_t`, `SPP_OsalPort_t` |
| `SPP_Module_functionName()` | Public functions (camelCase after underscore) | `SPP_Databank_getPacket()`, `ICM20948_configDmp()` |
| `p_name` | Pointer parameters and pointer variables | `p_cfg`, `p_handler`, `p_data` |
| `s_name` | Static module-level variables | `s_taskPool`, `s_initialized` |
| `RegXxx_t` | Register union types | `.value` for raw byte, `.bits` for named fields |

---

## SSH configuration (required for flash and debug)

Flash and debug connect to the remote ESP32-S3 board over SSH. The `ssh raspi` alias must be configured on your **host machine** — not inside the container. The container inherits the host's `~/.ssh` folder automatically (see `.devcontainer/devcontainer.json`).

### Create the SSH config on your host

Add the following to `~/.ssh/config` on your host machine:

```
Host raspi
    HostName <RASPI_IP>
    User <username>
    IdentityFile ~/.ssh/raspberrypi
    IdentitiesOnly yes
```

### Generate and install the key

```bash
ssh-keygen -t ed25519 -f ~/.ssh/raspberrypi
ssh-copy-id -i ~/.ssh/raspberrypi.pub <username>@<RASPI_IP>
```

### Verify

```bash
ssh raspi "echo connected"
```

### Why it works inside the container

`devcontainer.json` mounts `~/.ssh` from your host as read-only into `/tmp/host-ssh`. The `postCreateCommand` copies it to `/root/.ssh/` inside the container on startup. No extra configuration is needed inside the container.
