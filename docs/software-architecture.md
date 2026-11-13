# Software Architecture

This document describes the repository structure, the firmware layers, the build system, and how everything fits together in `solaris-v2`. It is written for students who know basic C but have not worked on embedded systems before.

---

## Repository structure

```
solaris-software/
├── solaris-v2/                # Active firmware
│   ├── main/                 # Application, FSM configuration
│   ├── compiler/             # ESP-IDF components
│   │   ├── spp/              # SPP library integration
│   │   └── spp_ports/        # Platform ports integration
│   └── spp/                  # SPP Git submodule
│       ├── core/             # Packet definitions, PUBSUB, core
│       ├── services/         # FSM, databank, sensors, telemetry
│       ├── hal/              # Hardware abstraction interfaces
│       ├── ports/            # Platform-specific implementations
│       ├── util/             # Shared utilities
│       └── tests/            # Unit tests
├── archive/                  # Legacy firmware
├── docs/                     # Architecture documentation
├── scripts/                  # Setup scripts
├── .devcontainer/            # Development environment
├── .vscode/                  # Editor and debugger configuration
└── .github/workflows/        # CI/CD

```

---


## Firmware layers

Solaris v2 uses a cooperative superloop architecture within ESP-IDF. Application execution is coordinated by a modular Finite State Machine (FSM).

The firmware is divided into the following layers:

1. **Application:** System startup and custom FSM configuration.
2. **FSM:** State transitions and periodic execution of the PUBSUB cycle.
3. **SPP Core:** Packet handling, PUBSUB routing and initialization.
4. **Services:** Sensor acquisition, telemetry, data logging and other functional modules.
5. **HAL:** Platform-independent interfaces for SPI, UART, GPIO, storage and time.
6. **Platform Ports:** Hardware-specific implementations of the HAL interfaces.

### Initialization

`app_main()` initializes the HAL and FSM, then enters the main execution loop through `APP_fsmLoop()`.

During the initialization state, the application registers producers and consumers. `SPP_CORE_init()` then initializes logging, the databank and PUBSUB, including the registered modules.

The custom FSM transitions the application to READY after successful core initialization and publishes an initialization-status packet.

### Data flow

The FSM periodically executes the PUBSUB cycle:

1. `SPP_SERVICES_PUBSUB_callProducers()` invokes registered producers.
2. Producers acquire data, obtain packets from the databank and publish them.
3. `publish()` copies packets into the internal circular buffer and returns the originals to the databank.
4. `SPP_SERVICES_PUBSUB_callConsumers()` dispatches one buffered packet to matching consumer mailboxes and invokes consumer processing.

Consumers execute in ascending numerical priority order. The implementation also supports producer-ready signaling for preemption.

### Packet format

SPP uses a fixed-size 64-byte packet containing:

- Primary header: version, APID, sequence and payload length.
- Secondary header: timestamp and drop counter.
- Payload: up to 50 bytes.
- CRC-16.

### Main services

- **Databank:** Static pool of 50 packets.
- **PUBSUB:** Producer/consumer registration, buffered routing and priority handling.
- **FSM:** Modular transition tables with internal and application-defined behavior.
- **BMP390:** Pressure, temperature and altitude acquisition.
- **ICM20948:** Accelerometer, gyroscope and quaternion telemetry.
- **Datalogger:** Binary packet logging directly to SD card sectors.
- **SX1262:** Radio telemetry consumer.
- **Logging:** Configurable diagnostic output.

Additional modules, including GNSS, are available in the source tree but are not necessarily enabled in the default build.

---

## Build system

### How ESP-IDF finds the SPP library

ESP-IDF discovers components by scanning directories listed in `EXTRA_COMPONENT_DIRS`. In `solaris-v2/CMakeLists.txt`, this is set to the `compiler/` directory:

```cmake
set(EXTRA_COMPONENT_DIRS compiler)
```

ESP-IDF then finds two components inside `compiler/`:

- `compiler/spp/` — compiles the SPP core and any enabled services
- `compiler/spp_ports/` — compiles the ESP32 HAL port

The actual SPP source code lives in `solaris-v2/spp/` as a **git submodule** — a separate repository embedded inside this one. After cloning `solaris-software`, initialise it:

```bash
git submodule update --init
```

### CMake options

`compiler/spp/CMakeLists.txt` exposes options to include or exclude individual services. All default to ON.


| Option | Default | Description |
|---|---|---|
| `SPP_SERVICE_BMP390` | ON | BMP390 sensor |
| `SPP_SERVICE_BMP390_SIM` | OFF | Simulated BMP390 data |
| `SPP_SERVICE_ICM20948` | ON | ICM20948 IMU |
| `SPP_SERVICE_DATALOGGER` | ON | SD card datalogger |
| `SPP_SX1262` | ON | SX1262 radio telemetry |
| `SPP_MAXM10M20B` | OFF | GNSS service |
| `SPP_SERVICE_UART` | OFF | UART consumer |
| `SPP_ENCRYPTION` | OFF | AES-128-GCM encryption |
| `SPP_DEBUG_PRINT` | OFF | Sensor debug output |
| `SPP_ANTENNA_DEVKIT` | OFF | Antenna development module |
| `SPP_HAL_ESP32` | ON | ESP32-S3 HAL port |

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

The HAL interface is defined in `spp/hal/hal.h` through `SPP_HalPort_t`.

The structure contains platform-independent function pointers grouped into five interfaces:

| Interface | Responsibilities |
|---|---|
| `SPP_HALSpi_t` | SPI initialization, transactions and device configuration |
| `SPP_HALUart_t` | UART initialization, transmission and reception |
| `SPP_HALGpio_t` | GPIO interrupts, digital input and output |
| `SPP_HALStorage_t` | Storage initialization and block writing |
| `SPP_HALTime_t` | Time measurement and delays |

The ESP32-S3 implementation is located in `spp/ports/hal/esp32/`.

The application registers the selected HAL port through `SPP_HAL_init()`.

---

## Return types and error handling


SPP functions that return `SPP_RetVal_t` report their execution status through the standard error codes. Callers should handle these results appropriately.

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

---

## Naming conventions

| Pattern | Used for | Example |
|---|---|---|
| `K_MODULE_*` | Constants and `#define` macros | `K_SPP_PKT_PAYLOAD_MAX`, `K_ICM20948_BANK0` |
| `K_MODULE_ENUM_VALUE` | Enum values | `K_SPP_LOG_VERBOSE` |
| `MODULE_TypeName_t` | Type definitions (struct, enum, typedef) | `SPP_Packet_t`, `BMP390_ServiceCtx_t` |
| `MODULE_SUBMODULE_functionName()` | Public functions | `SPP_SERVICES_DATABANK_getPacket()`, `SPP_SERVICES_BMP390_getProducerContract()` |
| `HAL_TARGET_functionName()` | HAL port internal functions | `SPP_PORTS_HAL_ESP32_spiBusInit()` |
| `p_name` | Pointer parameters and pointer variables | `p_cfg`, `p_handler`, `p_data` |
| `s_name` | Static module-level variables | `s_busInitialized`, `s_logBusy` |
| `k_name` | Static file-level constants | `k_tag` |

File names use **camelCase**: `halEsp32.c`, `macrosEsp32.h`, `returnTypes.h`.

---


## Packet identifiers (KPID)

SPP uses a 16-bit identifier field as a routing bitmask.

- `0x0001`: Reserved for logging.
- `0x0002`: Reserved for FSM telemetry.
- Remaining bits: Assigned to registered producers.

Each producer receives a unique bit during registration. Consumers subscribe using a bitmask combining the identifiers they wish to receive.

Identifier definitions are maintained in `spp/services/kpid.h`.

---

## Host machine setup

### Linux (Ubuntu / Debian / Fedora / Arch)

Run the setup script with sudo — it installs Git, Docker Engine, and VS Code/VS Codium, then configures the devcontainer:

```bash
sudo ./scripts/install-linux.sh
```

The script automatically configures Docker permissions and starts a new user session if necessary.

### Windows

```powershell
.\scripts\install-windows.ps1
```

---

## SSH configuration (remote flash and debug)

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
