#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "auv_description::simple_buoyancy" for configuration ""
set_property(TARGET auv_description::simple_buoyancy APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(auv_description::simple_buoyancy PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsimple_buoyancy.so"
  IMPORTED_SONAME_NOCONFIG "libsimple_buoyancy.so"
  )

list(APPEND _cmake_import_check_targets auv_description::simple_buoyancy )
list(APPEND _cmake_import_check_files_for_auv_description::simple_buoyancy "${_IMPORT_PREFIX}/lib/libsimple_buoyancy.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
