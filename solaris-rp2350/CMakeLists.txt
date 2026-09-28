# ----------------------------------------------------------------
# Solaris Firmware — Target: RP2350 (Raspberry Pi Pico 2 series)
# ----------------------------------------------------------------
cmake_minimum_required(VERSION 3.20)

# Configure RP2350 platform before importing SDK
set(PICO_PLATFORM rp2350 CACHE STRING "Target platform")
set(PICO_BOARD pico2 CACHE STRING "Target board")

# Include Pico SDK initialization script
include(pico_sdk_import.cmake)

project(solaris-rp2350 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialize Raspberry Pi Pico SDK
pico_sdk_init()

# ----------------------------------------------------------------
# Locate SPP library root
# ----------------------------------------------------------------
set(SPP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../solaris-v2/spp")
get_filename_component(SPP_ROOT "${SPP_ROOT}" ABSOLUTE)

if(NOT EXISTS "${SPP_ROOT}/core/core.h")
    message(FATAL_ERROR "SPP library not found at: ${SPP_ROOT}")
endif()
message(STATUS "SPP Root located at: ${SPP_ROOT}")

# ----------------------------------------------------------------
# SPP Service Options (matching solaris-v2 defaults)
# ----------------------------------------------------------------
option(SPP_SERVICE_BMP390     "Compile BMP390 pressure sensor service"             ON)
option(SPP_SERVICE_ICM20948   "Compile ICM20948 IMU service"                       OFF)
option(SPP_SERVICE_DATALOGGER "Compile datalogger (SD card) service"               OFF)
option(SPP_MAXM10M20B         "Compile GNSS (MAX-M10M) service"                    OFF)
option(SPP_ANTENNA_DEVKIT     "Compile antenna devkit (E22)"                       OFF)

# ----------------------------------------------------------------
# SPP Core Sources
# ----------------------------------------------------------------
set(SPP_CORE_SRCS
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

# ----------------------------------------------------------------
# SPP Service Sources
# ----------------------------------------------------------------
set(SPP_SERVICES_SRCS "")

if(SPP_SERVICE_BMP390)
    list(APPEND SPP_SERVICES_SRCS "${SPP_ROOT}/services/bmp390/bmp390.c")
endif()

if(SPP_SERVICE_ICM20948)
    list(APPEND SPP_SERVICES_SRCS "${SPP_ROOT}/services/icm20948/icm20948.c")
endif()

if(SPP_SERVICE_DATALOGGER)
    list(APPEND SPP_SERVICES_SRCS "${SPP_ROOT}/services/datalogger/datalogger.c")
endif()

if(SPP_MAXM10M20B)
    list(APPEND SPP_SERVICES_SRCS "${SPP_ROOT}/services/maxm10m20b/maxm10m20b.c")
endif()

# ----------------------------------------------------------------
# RP2350 HAL Port Sources
# ----------------------------------------------------------------
set(SPP_PORT_RP2350_SRCS
    "${SPP_ROOT}/ports/hal/rp2350/halRp2350.c"
)

# ----------------------------------------------------------------
# Target Application Executable
# ----------------------------------------------------------------
add_executable(solaris-rp2350
    main/main.c
    main/custom.c
    ${SPP_CORE_SRCS}
    ${SPP_SERVICES_SRCS}
    ${SPP_PORT_RP2350_SRCS}
)

# Include directories
target_include_directories(solaris-rp2350 PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/main"
    "${SPP_ROOT}/.."
    "${SPP_ROOT}/ports/hal/rp2350"
)

# Link Pico SDK hardware and standard libraries
target_link_libraries(solaris-rp2350
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_uart
    hardware_timer
)

# Enable USB CDC or UART output for stdio (default: UART enabled, USB disabled)
pico_enable_stdio_uart(solaris-rp2350 1)
pico_enable_stdio_usb(solaris-rp2350 0)

# Generate UF2, BIN, HEX and map files
pico_add_extra_outputs(solaris-rp2350)
