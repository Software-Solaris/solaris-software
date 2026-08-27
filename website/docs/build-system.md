SPP is built with CMake, and the whole point of the setup is that you rarely build all of it. Core files are always compiled, but every service and every platform port is behind a CMake option, off or on, so the final binary only carries what your project actually uses.

There are two separate places where this happens, because SPP gets built two different ways:

- spp/CMakeLists.txt is SPP's own, standalone build. It's what you use to build SPP by itself against the stub port, or to build and run the unit tests on a PC — no ESP32-S3, no ESP-IDF, no Docker container required.
- solaris-v2/compiler/spp/CMakeLists.txt and solaris-v2/compiler/spp_ports/CMakeLists.txt are ESP-IDF "components" that wrap SPP's sources for the real firmware build, the one you get from idf.py build inside the dev container. They locate the spp/ submodule automatically by walking up the directory tree, so you don't need to point them at anything by hand.

Both follow the same shape, so once you understand one, you understand the other.

## Core sources are unconditional

A handful of files always get compiled, no matter what — they're the mandatory infrastructure covered in the Core section: core.c, the PUBSUB service, the FSM, the DATABANK, the log service, the HAL contract, and a couple of small util files. In solaris-v2/compiler/spp/CMakeLists.txt that list looks like this:

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

There's no option() guarding any of these — if you need SPP at all, you need all of them.

## Services and everything else are opt-in

Everything on top of that core is guarded by a CMake option(), one per module, and an if() block that adds its source file(s) only when the option is ON:

```
option(SPP_SERVICE_BMP390     "Compile BMP390 pressure sensor service"              ON)
option(SPP_SERVICE_ICM20948   "Compile ICM20948 IMU service"                        ON)
option(SPP_SERVICE_DATALOGGER "Compile datalogger (SD card) service"                ON)
option(SPP_ENCRYPTION         "Compile AES-128-GCM encryption module"               OFF)

if(SPP_SERVICE_BMP390)
    list(APPEND SPP_SERVICE_FILES "${SPP_ROOT}/services/bmp390/bmp390.c")
endif()
```

The same pattern is used for HAL ports, in the spp_ports component:

```
option(SPP_HAL_ESP32 "Compile ESP32-S3 HAL (polling SPI)" ON)

if(SPP_HAL_ESP32)
    list(APPEND PORT_SRCS "${SPP_PORT_ROOT}/ports/hal/esp32/halEsp32.c")
endif()
```

And SPP's own standalone CMakeLists.txt uses it too, for picking which port to build against when you're not going through ESP-IDF at all:

```
option(SPP_PORT "Port to use: posix | freertos | baremetal" "posix")
```

If a service's option is OFF, its .c file is never added to the source list, so it's never compiled and never linked in — the resulting binary is smaller because that code simply isn't part of it, not because it's dead-stripped afterwards.

## Adding a new module

Whether it's a new sensor service or a new HAL port, the recipe is the same:

1. Add its .h and .c under spp/services/<name>/ (for a service) or spp/ports/hal/<target>/ (for a port).
2. Add one option() for it, next to the others.
3. Add one if() block that appends its source file(s) to the list when the option is ON.
4. If it's a service that needs to produce or consume data, register it with the PUBSUB service — see the Producers and Consumers sections of the PUBSUB chapter for the contract it needs to implement, and the FSM chapter for where that registration actually happens today.

That's it — you don't need to touch anything outside the module's own folder and its option()/if() pair.

## Removing a module to shrink the binary

Since every service and port is opt-in, shrinking the binary is just a matter of turning options off. You can do it two ways:

- At configure time, without touching any file, by passing the flag straight to idf.py:

```bash
idf.py build -DSPP_SERVICE_ICM20948=OFF
```

- Permanently, by flipping the option()'s default value in the CMakeLists.txt itself, if you know a given project will never need that module.

Either way, do a clean rebuild afterwards (idf.py fullclean && idf.py build) — CMake doesn't always notice on its own that a source file should be dropped from an existing build directory.

One thing to watch out for: these options are wired to a specific file path, and CMake doesn't check that the file still exists until it actually tries to compile it. If a module gets renamed or removed inside spp/ but the option() / if() block in solaris-v2/compiler still points at the old path, you'll get a build failure that has nothing to do with your own changes — it means the CMake side is out of sync with SPP itself. Grep for the file the error mentions under spp/services or spp/ports before assuming you broke something.
