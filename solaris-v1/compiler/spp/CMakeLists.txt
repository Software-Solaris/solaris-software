# ----------------------------------------------------------------
# spp — ESP-IDF component for the SPP core library
#
# Core (src/) is always compiled.
# Services are opt-in via CMake options (all ON by default):
#   SPP_SERVICE_BMP390     — BMP390 pressure sensor
#   SPP_SERVICE_ICM20948   — ICM20948 IMU
#   SPP_SERVICE_DATALOGGER — SD card datalogger
#
# To add a new service:
#   1. Add its .c files under spp/services/<name>/src/
#   2. Add one option() + one if-block in the services section below.
#
# To disable a service at build time (no source compiled at all):
#   idf.py build -DSPP_SERVICE_ICM20948=OFF
#
# Platform ports (OSAL + HAL implementations) live in spp_ports.
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
set(SPP_ROOT "")
foreach(p ${_candidates})
    if(EXISTS "${p}/include/spp/core/core.h")
        set(SPP_ROOT "${p}")
        break()
    endif()
endforeach()
if(SPP_ROOT STREQUAL "")
    message(FATAL_ERROR "spp: could not find spp/. Set SPP_ROOT.")
endif()
message(STATUS "spp: SPP_ROOT = ${SPP_ROOT}")

# Core library — always compiled
file(GLOB_RECURSE SPP_SRC_FILES "${SPP_ROOT}/src/*.c")

# ----------------------------------------------------------------
# Services — add one option() + if-block for each new service
# ----------------------------------------------------------------
option(SPP_SERVICE_BMP390     "Compile BMP390 pressure sensor service"  ON)
option(SPP_SERVICE_ICM20948   "Compile ICM20948 IMU service"            ON)
option(SPP_SERVICE_DATALOGGER "Compile datalogger (SD card) service"    ON)

set(SPP_SERVICE_FILES "")

if(SPP_SERVICE_BMP390)
    file(GLOB_RECURSE _svc "${SPP_ROOT}/services/bmp390/src/*.c")
    list(APPEND SPP_SERVICE_FILES ${_svc})
    message(STATUS "spp: BMP390 service       ON")
else()
    message(STATUS "spp: BMP390 service       OFF")
endif()

if(SPP_SERVICE_ICM20948)
    file(GLOB_RECURSE _svc "${SPP_ROOT}/services/icm20948/src/*.c")
    list(APPEND SPP_SERVICE_FILES ${_svc})
    message(STATUS "spp: ICM20948 service     ON")
else()
    message(STATUS "spp: ICM20948 service     OFF")
endif()

if(SPP_SERVICE_DATALOGGER)
    file(GLOB_RECURSE _svc "${SPP_ROOT}/services/datalogger/src/*.c")
    list(APPEND SPP_SERVICE_FILES ${_svc})
    message(STATUS "spp: DATALOGGER service   ON")
else()
    message(STATUS "spp: DATALOGGER service   OFF")
endif()

# Single include root: spp/core/, spp/hal/, spp/osal/, spp/services/, etc.
idf_component_register(
    SRCS         ${SPP_SRC_FILES} ${SPP_SERVICE_FILES}
    INCLUDE_DIRS "${SPP_ROOT}/include"
)
