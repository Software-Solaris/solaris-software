# ----------------------------------------------------------------
# spp_port_wrapper — ESP-IDF component for SPP platform ports
#
# Compiles the FreeRTOS OSAL + ESP32 HAL port and the baremetal
# OSAL + polling HAL port. Both are compiled in; the application
# selects the active port by registering the desired descriptor:
#   g_freertosOsalPort / g_esp32HalPort       — FreeRTOS
#   g_baremetalOsalPort / g_esp32BaremetalHalPort — baremetal
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
    if(EXISTS "${p}/ports/freertos/osal/osal_freertos.c")
        set(SPP_PORT_ROOT "${p}")
        break()
    endif()
endforeach()
if(SPP_PORT_ROOT STREQUAL "")
    message(FATAL_ERROR "spp_port_wrapper: could not find ports/freertos inside external/spp. Set SPP_ROOT.")
endif()
message(STATUS "spp_port_wrapper: SPP_PORT_ROOT = ${SPP_PORT_ROOT}")

# FreeRTOS OSAL + ESP32 HAL
set(FREERTOS_OSAL  "${SPP_PORT_ROOT}/ports/freertos/osal/osal_freertos.c")
set(ESP32_HAL      "${SPP_PORT_ROOT}/ports/freertos/hal/esp32/hal_esp32.c")
set(ESP32_HAL_INC  "${SPP_PORT_ROOT}/ports/freertos/hal/esp32/include")

# Baremetal OSAL + polling HAL
set(BM_OSAL        "${SPP_PORT_ROOT}/ports/baremetal/osal/osal_baremetal.c")
set(BM_HAL         "${SPP_PORT_ROOT}/ports/baremetal/hal/esp32/hal_esp32_baremetal.c")

idf_component_register(
    SRCS         ${FREERTOS_OSAL} ${ESP32_HAL} ${BM_OSAL} ${BM_HAL}
    INCLUDE_DIRS "${SPP_PORT_ROOT}/include" ${ESP32_HAL_INC}
    REQUIRES     spp_wrapper esp_driver_spi esp_driver_gpio
                 esp_driver_sdspi fatfs freertos esp_timer
)
