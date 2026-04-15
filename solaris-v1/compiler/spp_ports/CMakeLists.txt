# ----------------------------------------------------------------
# spp_ports — ESP-IDF component for SPP platform ports (ESP32)
#
# OSAL and HAL ports are opt-in via CMake options (all ON by default):
#   SPP_OSAL_FREERTOS    — FreeRTOS OSAL  → g_freertosOsalPort
#   SPP_OSAL_BAREMETAL   — Baremetal OSAL → g_baremetalOsalPort
#   SPP_HAL_ESP32        — ESP32 HAL (FreeRTOS SPI)  → g_esp32HalPort
#   SPP_HAL_ESP32_BM     — ESP32 HAL (polling SPI)   → g_esp32BaremetalHalPort
#
# To disable a port at build time:
#   idf.py build -DSPP_OSAL_FREERTOS=OFF
#
# To add a new port, add one option() + if-block in the relevant
# section below, then extend INCLUDE_DIRS if the port has headers.
# ----------------------------------------------------------------

# Locate spp
set(_bases "${CMAKE_CURRENT_LIST_DIR};${PROJECT_SOURCE_DIR};${CMAKE_SOURCE_DIR}")
set(_candidates "")
if(SPP_ROOT)
    list(APPEND _candidates "${SPP_ROOT}")
endif()
if(DEFINED ENV{SPP_ROOT})
    list(APPEND _candidates "$ENV{SPP_ROOT}")
endif()
foreach(b ${_bases})
    get_filename_component(babs "${b}" ABSOLUTE)
    set(cur "${babs}")
    foreach(_n 0 1 2 3 4 5)
        if(EXISTS "${cur}/spp")
            list(APPEND _candidates "${cur}/spp")
        endif()
        get_filename_component(cur "${cur}/.." ABSOLUTE)
    endforeach()
endforeach()
list(REMOVE_DUPLICATES _candidates)
set(SPP_PORT_ROOT "")
foreach(p ${_candidates})
    if(EXISTS "${p}/ports/osal/freertos/osal_freertos.c")
        set(SPP_PORT_ROOT "${p}")
        break()
    endif()
endforeach()
if(SPP_PORT_ROOT STREQUAL "")
    message(FATAL_ERROR "spp_ports: could not locate ports/osal/freertos inside spp/.")
endif()
message(STATUS "spp_ports: SPP_PORT_ROOT = ${SPP_PORT_ROOT}")

# ----------------------------------------------------------------
# OSAL ports — add one option() + if-block for each new OS port
# ----------------------------------------------------------------
option(SPP_OSAL_FREERTOS  "Compile FreeRTOS OSAL port"   ON)
option(SPP_OSAL_BAREMETAL "Compile baremetal OSAL port"  ON)

set(PORT_SRCS "")

if(SPP_OSAL_FREERTOS)
    list(APPEND PORT_SRCS "${SPP_PORT_ROOT}/ports/osal/freertos/osal_freertos.c")
    message(STATUS "spp_ports: OSAL FreeRTOS   ON")
else()
    message(STATUS "spp_ports: OSAL FreeRTOS   OFF")
endif()

if(SPP_OSAL_BAREMETAL)
    list(APPEND PORT_SRCS "${SPP_PORT_ROOT}/ports/osal/baremetal/osal_baremetal.c")
    message(STATUS "spp_ports: OSAL Baremetal  ON")
else()
    message(STATUS "spp_ports: OSAL Baremetal  OFF")
endif()

# ----------------------------------------------------------------
# HAL ports — add one option() + if-block for each new target
# ----------------------------------------------------------------
option(SPP_HAL_ESP32    "Compile ESP32 HAL (FreeRTOS SPI)"  ON)
option(SPP_HAL_ESP32_BM "Compile ESP32 HAL (polling SPI)"   ON)

set(HAL_ESP32_INC "${SPP_PORT_ROOT}/ports/hal/esp32/include")

if(SPP_HAL_ESP32)
    list(APPEND PORT_SRCS "${SPP_PORT_ROOT}/ports/hal/esp32/hal_esp32.c")
    message(STATUS "spp_ports: HAL ESP32       ON")
else()
    message(STATUS "spp_ports: HAL ESP32       OFF")
endif()

if(SPP_HAL_ESP32_BM)
    list(APPEND PORT_SRCS "${SPP_PORT_ROOT}/ports/hal/esp32/hal_esp32_baremetal.c")
    message(STATUS "spp_ports: HAL ESP32 BM    ON")
else()
    message(STATUS "spp_ports: HAL ESP32 BM    OFF")
endif()

idf_component_register(
    SRCS         ${PORT_SRCS}
    INCLUDE_DIRS "${SPP_PORT_ROOT}/include" ${HAL_ESP32_INC}
    REQUIRES     spp esp_driver_spi esp_driver_gpio
                 esp_driver_sdspi fatfs freertos esp_timer
)
