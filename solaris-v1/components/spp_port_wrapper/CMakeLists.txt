# ----------------------------------------------------------------
# spp_port_wrapper — ESP-IDF component for SPP platform ports
#
# OSAL and HAL are now in separate trees:
#   ports/osal/<os>/        — OS abstraction (freertos, baremetal, posix)
#   ports/hal/<target>/     — hardware abstraction (esp32, stub)
#
# Both FreeRTOS and baremetal variants are compiled in. The application
# selects the active port by registering the desired descriptors:
#   FreeRTOS  → g_freertosOsalPort   + g_esp32HalPort
#   Baremetal → g_baremetalOsalPort  + g_esp32BaremetalHalPort
# ----------------------------------------------------------------

# Locate external/spp
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
        if(EXISTS "${cur}/external/spp")
            list(APPEND _candidates "${cur}/external/spp")
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
    message(FATAL_ERROR "spp_port_wrapper: could not locate ports/osal/freertos inside external/spp.")
endif()
message(STATUS "spp_port_wrapper: SPP_PORT_ROOT = ${SPP_PORT_ROOT}")

# OSAL ports
set(OSAL_FREERTOS  "${SPP_PORT_ROOT}/ports/osal/freertos/osal_freertos.c")
set(OSAL_BAREMETAL "${SPP_PORT_ROOT}/ports/osal/baremetal/osal_baremetal.c")

# HAL ports (ESP32)
set(HAL_ESP32_FREERTOS  "${SPP_PORT_ROOT}/ports/hal/esp32/hal_esp32.c")
set(HAL_ESP32_BAREMETAL "${SPP_PORT_ROOT}/ports/hal/esp32/hal_esp32_baremetal.c")
set(HAL_ESP32_INC       "${SPP_PORT_ROOT}/ports/hal/esp32/include")

idf_component_register(
    SRCS         ${OSAL_FREERTOS} ${OSAL_BAREMETAL}
                 ${HAL_ESP32_FREERTOS} ${HAL_ESP32_BAREMETAL}
    INCLUDE_DIRS "${SPP_PORT_ROOT}/include" ${HAL_ESP32_INC}
    REQUIRES     spp_wrapper esp_driver_spi esp_driver_gpio
                 esp_driver_sdspi fatfs freertos esp_timer
)
