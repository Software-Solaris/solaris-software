# ----------------------------------------------------------------
# spp — ESP-IDF component for the SPP core library
#
# Core sources are always compiled.
# Services are opt-in via CMake options (all ON by default):
#   SPP_SERVICE_BMP390     — BMP390 pressure sensor
#   SPP_SERVICE_ICM20948   — ICM20948 IMU
#   SPP_SERVICE_DATALOGGER — SD card datalogger
#
# To add a new service:
#   1. Add its .h and .c under spp/services/<name>/
#   2. Add one option() + one if-block in the services section below.
#
# To disable a service at build time:
#   idf.py build -DSPP_SERVICE_ICM20948=OFF
#
# Platform ports (OSAL + HAL implementations) live in compiler/spp_ports.
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
set(SPP_ROOT "")
foreach(p ${_candidates})
    if(EXISTS "${p}/core/core.h")
        set(SPP_ROOT "${p}")
        break()
    endif()
endforeach()
if(SPP_ROOT STREQUAL "")
    message(FATAL_ERROR "spp: could not find spp/. Set SPP_ROOT.")
endif()
message(STATUS "spp: SPP_ROOT = ${SPP_ROOT}")

# ----------------------------------------------------------------
# Core sources — always compiled
# ----------------------------------------------------------------
set(SPP_SRC_FILES
    "${SPP_ROOT}/core/core.c"
    "${SPP_ROOT}/core/error.c"
    "${SPP_ROOT}/hal/dispatch.c"
    "${SPP_ROOT}/services/service.c"
    "${SPP_ROOT}/services/databank/databank.c"
    "${SPP_ROOT}/services/pubsub/pubsub.c"
    "${SPP_ROOT}/services/log/log.c"
    "${SPP_ROOT}/util/crc.c"
)

# ----------------------------------------------------------------
# Services — add one option() + if-block for each new service
# ----------------------------------------------------------------
option(SPP_SERVICE_BMP390     "Compile BMP390 pressure sensor service"  ON)
option(SPP_SERVICE_ICM20948   "Compile ICM20948 IMU service"            ON)
option(SPP_SERVICE_DATALOGGER "Compile datalogger (SD card) service"    ON)
option(SPP_DEBUG_PRINT        "Print sensor values to stdout via printf" ON)

set(SPP_SERVICE_FILES "")

if(SPP_SERVICE_BMP390)
    list(APPEND SPP_SERVICE_FILES "${SPP_ROOT}/services/bmp390/bmp390.c")
    message(STATUS "spp: BMP390 service       ON")
else()
    message(STATUS "spp: BMP390 service       OFF")
endif()

if(SPP_SERVICE_ICM20948)
    list(APPEND SPP_SERVICE_FILES "${SPP_ROOT}/services/icm20948/icm20948.c")
    message(STATUS "spp: ICM20948 service     ON")
else()
    message(STATUS "spp: ICM20948 service     OFF")
endif()

if(SPP_SERVICE_DATALOGGER)
    list(APPEND SPP_SERVICE_FILES "${SPP_ROOT}/services/datalogger/datalogger.c")
    message(STATUS "spp: DATALOGGER service   ON")
else()
    message(STATUS "spp: DATALOGGER service   OFF")
endif()

if(SPP_DEBUG_PRINT)
    list(APPEND SPP_SERVICE_FILES)
    message(STATUS "spp: DEBUG_PRINT          ON")
else()
    message(STATUS "spp: DEBUG_PRINT          OFF")
endif()

# Include root is the parent of spp/ so #include "spp/core/packet.h" resolves correctly.
idf_component_register(
    SRCS         ${SPP_SRC_FILES} ${SPP_SERVICE_FILES}
    INCLUDE_DIRS "${SPP_ROOT}/.."
)

if(SPP_DEBUG_PRINT)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC SPP_DEBUG_PRINT=1)
endif()
