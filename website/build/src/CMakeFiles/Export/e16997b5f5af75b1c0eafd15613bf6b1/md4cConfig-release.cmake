#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "md4c::md4c" for configuration "Release"
set_property(TARGET md4c::md4c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(md4c::md4c PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmd4c.so.0.5.3"
  IMPORTED_SONAME_RELEASE "libmd4c.so.0"
  )

list(APPEND _cmake_import_check_targets md4c::md4c )
list(APPEND _cmake_import_check_files_for_md4c::md4c "${_IMPORT_PREFIX}/lib/libmd4c.so.0.5.3" )

# Import target "md4c::md4c-html" for configuration "Release"
set_property(TARGET md4c::md4c-html APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(md4c::md4c-html PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmd4c-html.so.0.5.3"
  IMPORTED_SONAME_RELEASE "libmd4c-html.so.0"
  )

list(APPEND _cmake_import_check_targets md4c::md4c-html )
list(APPEND _cmake_import_check_files_for_md4c::md4c-html "${_IMPORT_PREFIX}/lib/libmd4c-html.so.0.5.3" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
