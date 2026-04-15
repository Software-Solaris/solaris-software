# ----------------------------------------------------------------
# spp_wrapper — ESP-IDF component for SPP core + services
#
# Compiles everything in spp/src/ and exposes spp/include/ as the
# public include path.  Platform-specific ports (osal, hal) come
# from spp_port_wrapper.
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
set(SPP_ROOT "")
foreach(p ${_candidates})
    if(EXISTS "${p}/include/spp/core/core.h")   # Verify new structure
        set(SPP_ROOT "${p}")
        break()
    endif()
endforeach()
if(SPP_ROOT STREQUAL "")
    message(FATAL_ERROR "spp_wrapper: could not find external/spp (new structure). Set SPP_ROOT.")
endif()
message(STATUS "spp_wrapper: SPP_ROOT = ${SPP_ROOT}")

# Collect all .c files under src/ — no hal or osal ports (those come from spp_port_wrapper)
file(GLOB_RECURSE SPP_SRC_FILES "${SPP_ROOT}/src/*.c")

# Also include services/ sensor service sources if present
file(GLOB_RECURSE SPP_SERVICE_FILES "${SPP_ROOT}/services/*.c")

set(SPP_ALL_SOURCES ${SPP_SRC_FILES} ${SPP_SERVICE_FILES})

# Single include root: spp/include/ — all #include "spp/..." paths resolve here
set(SPP_INCLUDE_DIR "${SPP_ROOT}/include")

# Also expose services include dirs
file(GLOB_RECURSE SPP_SERVICE_HEADERS "${SPP_ROOT}/services/*/include")
set(SPP_EXTRA_INCLUDES "")
foreach(svc_inc ${SPP_SERVICE_HEADERS})
    if(IS_DIRECTORY "${svc_inc}")
        list(APPEND SPP_EXTRA_INCLUDES "${svc_inc}")
    endif()
endforeach()

idf_component_register(
    SRCS         ${SPP_ALL_SOURCES}
    INCLUDE_DIRS ${SPP_INCLUDE_DIR} ${SPP_EXTRA_INCLUDES}
)
