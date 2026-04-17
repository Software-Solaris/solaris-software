# ----------------------------------------------------------------
# spp_ports — ESP-IDF component for SPP platform ports (ESP32)
#
# HAL ports live in spp/ports/hal/<target>/
#
# Available ports:
#   SPP_HAL_ESP32  — ESP32-S3 HAL (polling SPI, no FreeRTOS)
#
# To add a new port:
#   HAL: add ports/hal/<target>/hal_<target>.c + option() + if-block below
# ----------------------------------------------------------------

# ----------------------------------------------------------------
# Locate spp root
# ----------------------------------------------------------------
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
    if(EXISTS "${p}/ports/hal/esp32/hal_esp32.c")
        set(SPP_PORT_ROOT "${p}")
        break()
    endif()
endforeach()
if(SPP_PORT_ROOT STREQUAL "")
    message(FATAL_ERROR "spp_ports: could not locate spp/ports/. Set SPP_ROOT.")
endif()
message(STATUS "spp_ports: SPP_PORT_ROOT = ${SPP_PORT_ROOT}")

# ----------------------------------------------------------------
# HAL ports — add one option() + if-block for each new target
# ----------------------------------------------------------------
option(SPP_HAL_ESP32 "Compile ESP32-S3 HAL (polling SPI)" ON)

set(PORT_SRCS "")

if(SPP_HAL_ESP32)
    list(APPEND PORT_SRCS "${SPP_PORT_ROOT}/ports/hal/esp32/hal_esp32.c")
    message(STATUS "spp_ports: HAL ESP32       ON")
else()
    message(STATUS "spp_ports: HAL ESP32       OFF")
endif()

# Include root is the parent of spp/ for SPP headers.
# ports/hal/esp32/ is added so macros_esp32.h resolves for HAL sources.
idf_component_register(
    SRCS         ${PORT_SRCS}
    INCLUDE_DIRS "${SPP_PORT_ROOT}/.." "${SPP_PORT_ROOT}/ports/hal/esp32"
    REQUIRES     spp esp_driver_spi esp_driver_gpio
                 esp_driver_sdspi fatfs esp_timer
)
