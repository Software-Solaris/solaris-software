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
│   │   └── spp_ports/           # Compiles platform HAL ports
│   └── spp/                     # SPP library (git submodule)
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
├── archive/                     # Legacy firmware (v0.1, v0.2)
├── docs/                        # Architecture documentation
├── scripts/                     # Setup scripts (Linux, Windows)
├── .devcontainer/               # Docker build environment
├── .vscode/                     # VS Code tasks and debug config
└── .github/workflows/           # CI/CD pipeline
```

---

## Firmware layers

Solaris v1 runs on a **bare-metal superloop** — there is no RTOS, no tasks, no queues, and no scheduler. Each layer only depends on the one below it.

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
              Hardware: ESP32-S3 · SPI bus
```

### Data flow (bare-metal superloop pattern)

```
ISR sets volatile drdyFlag
  → superloop detects flag
    → ServiceTask()
        → SPP_SERVICES_DATABANK_getPacket()
        → SPP_SERVICES_DATABANK_packetData(pkt, apid, seq, data, len)
              fills all headers, copies payload, computes CRC-16
        → SPP_SERVICES_PUBSUB_publish(pkt)
              dispatches to all matching subscribers synchronously
              then calls SPP_SERVICES_DATABANK_returnPacket(pkt) automatically
```

### HAL — Hardware Abstraction Layer

HAL defines *what* hardware operations exist (SPI, GPIO, SD card storage) but contains no implementation. The headers in `spp/hal/` describe the `SPP_HalPort_t` struct — a table of function pointers. Platform-specific code fills this struct in `spp/ports/hal/`.

| Function | What it does |
|---|---|
| `SPP_HAL_spiBusInit()` | Initialises SPI2_HOST (MISO 47, MOSI 38, CLK 48) |
| `SPP_HAL_spiDeviceInit()` | Registers a device on the bus (ICM at CS 21, BMP at CS 18) |
| `SPP_HAL_spiTransmit()` | Full-duplex SPI transaction; handles R/W bit per device |
| `SPP_HAL_gpioConfigInterrupt()` | Configures a GPIO pin for edge-triggered interrupts |
| `SPP_HAL_gpioRegisterIsr()` | Attaches an ISR that sets a `volatile spp_bool_t` flag |
| `SPP_HAL_storageMount()` | Mounts the SD card via FATFS + SDSPI |
| `SPP_HAL_storageUnmount()` | Unmounts the SD card safely |
| `SPP_HAL_getTimeMs()` | Returns the current time in milliseconds |

GPIO ISRs are minimal: they set a `volatile spp_bool_t` flag and return immediately. No RTOS calls, no yield.

### SPP Core

The Solaris Packet Protocol defines a common packet format for all sensor data:

```
SPP_Packet_t
├── Primary header    APID · sequence · payload_len
├── Secondary header  timestamp_ms · drop_counter
├── Payload           up to 48 bytes of sensor data
└── CRC-16/CCITT
```

**Built-in services on top of Core:**

**Databank** — static pool of 5 packets. Call `SPP_SERVICES_DATABANK_getPacket()` to lease one, fill it with `SPP_SERVICES_DATABANK_packetData()`, then publish it. No `malloc` is ever called.

**Pub/Sub** — synchronous publish-subscribe router. Subscribers register with `SPP_SERVICES_PUBSUB_subscribe(apid, handler, ctx)`. `SPP_SERVICES_PUBSUB_publish(pkt)` dispatches to all matching handlers synchronously and returns the packet to the databank automatically. `K_SPP_APID_ALL (0xFFFF)` subscribes to every packet.

**Logging** — `SPP_LOGE/W/I/D/V(tag, fmt, ...)` level-filtered macros. Output registered via `SPP_SERVICES_LOG_registerOutput()`. In `main.c` the log output is bridged into pub/sub so log messages are published as `K_SPP_APID_LOG (0x0001)` packets and reach the SD card logger.

**Service registry** — up to 16 services registered via `SPP_SERVICES_register()`. `SPP_SERVICES_initAll()` and `SPP_SERVICES_startAll()` iterate the registry and call each service's `init` and `start` callbacks.

### Sensor services

**BMP390** (`spp/services/bmp390/`)
- Barometer: pressure, temperature, altitude (APID `0x0101`)
- Data-ready interrupt via GPIO → sets `bmpData.drdyFlag`
- `SPP_SERVICES_BMP390_serviceTask()` called from superloop when flag is set
- Publishes 3×float payload (pressure, temperature, altitude)

**ICM20948** (`spp/services/icm20948/`)
- 9-axis IMU: accelerometer, gyroscope, magnetometer AK09916 (APID `0x0201`)
- DMP (Digital Motion Processor) computes quaternions on-chip
- Data-ready interrupt → sets `icmData.drdyFlag`
- `SPP_SERVICES_ICM20948_serviceTask()` drains DMP FIFO, publishes 9×float (ax/ay/az/gx/gy/gz/mx/my/mz)

**Datalogger** (`spp/services/datalogger/`)
- Pub/sub subscriber. Writes one line per packet to SD card:
  - `K_SPP_APID_LOG` packets: payload string + `\n`
  - Sensor packets: `ts=<ms> apid=0x<X> seq=<N> len=<N> payload_hex=...\n`
- Flushes every N packets to limit data loss on power cut

---

## Build system

### How ESP-IDF finds the SPP library

ESP-IDF discovers components by scanning directories listed in `EXTRA_COMPONENT_DIRS`. In `solaris-v1/CMakeLists.txt`, this is set to the `compiler/` directory:

```cmake
set(EXTRA_COMPONENT_DIRS compiler)
```

ESP-IDF then finds two components inside `compiler/`:

- `compiler/spp/` — compiles the SPP core and any enabled services
- `compiler/spp_ports/` — compiles the ESP32 HAL port

The actual SPP source code lives in `solaris-v1/spp/` as a **git submodule** — a separate repository embedded inside this one. After cloning `solaris-software`, initialise it:

```bash
git submodule update --init
```

### CMake options

`compiler/spp/CMakeLists.txt` exposes options to include or exclude individual services. All default to ON.

| Option | Default | What it enables |
|---|---|---|
| `SPP_SERVICE_BMP390` | ON | BMP390 pressure sensor service |
| `SPP_SERVICE_ICM20948` | ON | ICM20948 IMU service |
| `SPP_SERVICE_DATALOGGER` | ON | SD card datalogger service |
| `SPP_HAL_ESP32` | ON | ESP32-S3 HAL (polling SPI, no FreeRTOS) |

To disable a service at build time:

```bash
idf.py build -DSPP_SERVICE_ICM20948=OFF
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
        spp
)
```

### Step 3 — Done

ESP-IDF picks up the new component automatically because `compiler/` is already listed in `EXTRA_COMPONENT_DIRS`. No changes to the top-level `CMakeLists.txt` are needed.

---

## HAL contract

The full port contract is defined in `spp/hal/port.h`.

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
| `delayMs` | Busy-wait delay |

---

## Return types and error handling

All SPP functions return `SPP_RetVal_t`. Check the return value of every call.

| Value | Meaning |
|---|---|
| `K_SPP_OK` | Success |
| `K_SPP_ERROR` | Generic error |
| `K_SPP_NOT_ENOUGH_PACKETS` | Databank pool exhausted |
| `K_SPP_NULL_PACKET` | Packet pointer was NULL |
| `K_SPP_ERROR_ALREADY_INITIALIZED` | Component already initialised |
| `K_SPP_ERROR_NULL_POINTER` | A required pointer argument was NULL |
| `K_SPP_ERROR_NOT_INITIALIZED` | Component was not initialised before use |
| `K_SPP_ERROR_INVALID_PARAMETER` | An argument value is out of range or nonsensical |
| `K_SPP_ERROR_ON_SPI_TRANSACTION` | The SPI bus returned an error |
| `K_SPP_ERROR_TIMEOUT` | A blocking operation exceeded its timeout |
| `K_SPP_ERROR_NO_PORT` | A HAL port was not registered before use |
| `K_SPP_ERROR_REGISTRY_FULL` | The service registry has no free slots |

For rich error context (source file and line number), use `SPP_CORE_errToString()` and `SPP_CORE_errToStringR()`.

---

## Naming conventions

| Pattern | Used for | Example |
|---|---|---|
| `K_MODULE_*` | Constants and `#define` macros | `K_SPP_PKT_PAYLOAD_MAX`, `K_ICM20948_BANK0` |
| `K_MODULE_ENUM_VALUE` | Enum values | `K_SPP_LOG_VERBOSE` |
| `MODULE_TypeName_t` | Type definitions (struct, enum, typedef) | `SPP_Packet_t`, `BMP390_ServiceCtx_t` |
| `MODULE_SUBMODULE_functionName()` | Public functions | `SPP_SERVICES_DATABANK_getPacket()`, `SPP_SERVICES_BMP390_serviceTask()` |
| `HAL_TARGET_functionName()` | HAL port internal functions | `SPP_PORTS_HAL_ESP32_spiBusInit()` |
| `p_name` | Pointer parameters and pointer variables | `p_cfg`, `p_handler`, `p_data` |
| `s_name` | Static module-level variables | `s_busInitialized`, `s_logBusy` |
| `k_name` | Static file-level constants | `k_tag` |

File names use **camelCase**: `halEsp32.c`, `macrosEsp32.h`, `returnTypes.h`.

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

## Host machine setup

### Linux (Ubuntu / Debian / Fedora / Arch)

Run the setup script with sudo — it installs Git, Docker Engine, and VS Code/VS Codium, then configures the devcontainer:

```bash
sudo ./scripts/install-linux.sh
```

After the script finishes, add your user to the `docker` group so you can run Docker without sudo:

```bash
sudo usermod -aG docker $USER
```

Then **log out and back in** (or run `newgrp docker`) for the group change to take effect. Verify with:

```bash
docker run --rm hello-world
```

### Windows

```powershell
.\scripts\install-windows.ps1
```

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

`devcontainer.json` mounts `~/.ssh` from your host as read-only into `/tmp/host-ssh`. The `postCreateCommand` copies it to `/root/.ssh/` inside the container on startup.
