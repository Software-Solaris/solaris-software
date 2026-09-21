SPP is built with CMake. Core files are always compiled, but each service and platform port is controlled by a CMake option, so the resulting binary only includes the modules a given project actually uses.

The build is defined in two separate locations, because SPP supports two distinct build paths:

- `spp/CMakeLists.txt` is SPP's standalone build. It is used to build SPP on its own against the stub port, or to build and run the unit tests on a PC, without an ESP32-S3 target, ESP-IDF, or a Docker container.
- `solaris-v2/compiler/spp/CMakeLists.txt` and `solaris-v2/compiler/spp_ports/CMakeLists.txt` are ESP-IDF components that wrap SPP's sources for the firmware build produced by `idf.py build` inside the development container. They locate the `spp/` submodule automatically by walking up the directory tree.

Both follow the same structure.

## Core sources are unconditional

A fixed set of files is always compiled: the mandatory infrastructure described in the Core section — `core.c`, the PUBSUB service, the FSM, the databank, the log service, the HAL contract, and a small number of utility files. In `solaris-v2/compiler/spp/CMakeLists.txt`, that list is defined as follows:

```
set(SPP_SRC_FILES
    "${SPP_ROOT}/core/core.c"
    "${SPP_ROOT}/core/pubsub/pubsub.c"
    "${SPP_ROOT}/core/commonbit.c"
    "${SPP_ROOT}/hal/hal.c"
    "${SPP_ROOT}/hal/spi/spi.c"
    "${SPP_ROOT}/hal/uart/uart.c"
    "${SPP_ROOT}/hal/gpio/gpio.c"
    "${SPP_ROOT}/hal/storage/storage.c"
    "${SPP_ROOT}/hal/time/time.c"
    "${SPP_ROOT}/services/service.c"
    "${SPP_ROOT}/services/databank/databank.c"
    "${SPP_ROOT}/services/log/log.c"
    "${SPP_ROOT}/services/fsm/fsm.c"
    "${SPP_ROOT}/util/crc.c"
)
```

None of these files are guarded by an `option()`; any project that uses SPP requires all of them.

## Services and ports are opt-in

Everything beyond the core is guarded by one CMake `option()` per module, with an `if()` block that appends the corresponding source file(s) only when the option is enabled:

```
option(SPP_SERVICE_BMP390     "Compile BMP390 pressure sensor service"              ON)
option(SPP_SERVICE_ICM20948   "Compile ICM20948 IMU service"                        ON)
option(SPP_SERVICE_DATALOGGER "Compile datalogger (SD card) service"                ON)
option(SPP_ENCRYPTION         "Compile AES-128-GCM encryption module"               OFF)

if(SPP_SERVICE_BMP390)
    list(APPEND SPP_SERVICE_FILES "${SPP_ROOT}/services/bmp390/bmp390.c")
endif()
```

The same pattern applies to HAL ports, in the `spp_ports` component:

```
option(SPP_HAL_ESP32 "Compile ESP32-S3 HAL (polling SPI)" ON)

if(SPP_HAL_ESP32)
    list(APPEND PORT_SRCS "${SPP_PORT_ROOT}/ports/hal/esp32/halEsp32.c")
endif()
```

SPP's standalone `CMakeLists.txt` uses the same mechanism to select which port to build against outside of ESP-IDF:

```
option(SPP_PORT "Port to use: posix | freertos | baremetal" "posix")
```

When a service's option is set to `OFF`, its source file is never added to the build, so it is neither compiled nor linked — the resulting binary is smaller because that code is absent from the build, not because it was removed afterward.

## Adding a new module

The procedure is the same for a new sensor service or a new HAL port:

1. Add its header and source files under `spp/services/<name>/` (for a service) or `spp/ports/hal/<target>/` (for a port).
2. Add an `option()` for it, alongside the existing ones.
3. Add an `if()` block that appends its source file(s) to the list when the option is enabled.
4. If the service produces or consumes data, register it with the PUBSUB service. See the Producers and Consumers sections of the PUBSUB chapter for the contract it must implement, and the FSM chapter for where that registration currently takes place.

No changes outside the module's own directory and its `option()`/`if()` pair are required.

## Removing a module to reduce binary size

Since every service and port is opt-in, reducing binary size is a matter of disabling the corresponding options. This can be done in two ways:

- At configure time, without modifying any file, by passing the flag to `idf.py`:

```bash
idf.py build -DSPP_SERVICE_ICM20948=OFF
```

- Permanently, by changing the option's default value in the CMakeLists.txt, for projects that will never require a given module.

In either case, perform a clean rebuild afterward (`idf.py fullclean && idf.py build`): CMake does not always detect that a source file should be removed from an existing build directory.

These options reference a specific file path, and CMake does not verify that the file exists until it attempts to compile it. If a module is renamed or removed within `spp/` while the corresponding `option()`/`if()` block in `solaris-v2/compiler` still references the old path, the resulting build failure is unrelated to any other change being made — it indicates that the CMake configuration is out of sync with SPP. Search for the file named in the error under `spp/services` or `spp/ports` before assuming the failure originates elsewhere.
