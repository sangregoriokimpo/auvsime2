#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "auve1_description::wasd_body_thrust_plugin" for configuration ""
set_property(TARGET auve1_description::wasd_body_thrust_plugin APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(auve1_description::wasd_body_thrust_plugin PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libwasd_body_thrust_plugin.so"
  IMPORTED_SONAME_NOCONFIG "libwasd_body_thrust_plugin.so"
  )

list(APPEND _cmake_import_check_targets auve1_description::wasd_body_thrust_plugin )
list(APPEND _cmake_import_check_files_for_auve1_description::wasd_body_thrust_plugin "${_IMPORT_PREFIX}/lib/libwasd_body_thrust_plugin.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
