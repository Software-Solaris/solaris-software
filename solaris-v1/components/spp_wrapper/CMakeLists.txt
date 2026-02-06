# Detecta SPP_ROOT igual que antes (dejamos tu lógica tal cual)
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
  if(EXISTS "${p}")
    set(SPP_ROOT "${p}")
    break()
  endif()
endforeach()

if(SPP_ROOT STREQUAL "")
  message(FATAL_ERROR "No se encontró external/spp. Fija SPP_ROOT.")
endif()


# ---------------------------------------------------------
# SIMPLÍSIMO:
#   INCLUYE *.c SOLO DEL DIRECTORIO RAÍZ DE external/spp
#   NO coge nada que esté dentro de hal/ ni osal/
# ---------------------------------------------------------

# Solo .c directamente en external/spp (sin recursión)
file(GLOB_RECURSE SPP_EXT_SOURCES
    "${SPP_ROOT}/*.c"
)


# No cogemos ni external/spp/hal/*.c
# ni external/spp/osal/*.c
# ni ningún subdirectorio
# por diseño.

# Headers (estos sí recursivos)
file(GLOB_RECURSE SPP_EXT_HEADERS "${SPP_ROOT}/*.h")
set(SPP_EXT_INCLUDE_DIRS "${SPP_ROOT}")
foreach(h ${SPP_EXT_HEADERS})
  get_filename_component(d "${h}" DIRECTORY)
  list(APPEND SPP_EXT_INCLUDE_DIRS "${d}")
endforeach()
list(REMOVE_DUPLICATES SPP_EXT_INCLUDE_DIRS)


idf_component_register(
  SRCS         ${SPP_EXT_SOURCES}
  INCLUDE_DIRS ${SPP_EXT_INCLUDE_DIRS}
)
