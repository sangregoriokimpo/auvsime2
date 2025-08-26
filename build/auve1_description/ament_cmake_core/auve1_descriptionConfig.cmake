# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_auve1_description_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED auve1_description_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(auve1_description_FOUND FALSE)
  elseif(NOT auve1_description_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(auve1_description_FOUND FALSE)
  endif()
  return()
endif()
set(_auve1_description_CONFIG_INCLUDED TRUE)

# output package information
if(NOT auve1_description_FIND_QUIETLY)
  message(STATUS "Found auve1_description: 0.0.0 (${auve1_description_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'auve1_description' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT auve1_description_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(auve1_description_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${auve1_description_DIR}/${_extra}")
endforeach()
