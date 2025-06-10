# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_oak_camera_integration_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED oak_camera_integration_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(oak_camera_integration_FOUND FALSE)
  elseif(NOT oak_camera_integration_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(oak_camera_integration_FOUND FALSE)
  endif()
  return()
endif()
set(_oak_camera_integration_CONFIG_INCLUDED TRUE)

# output package information
if(NOT oak_camera_integration_FIND_QUIETLY)
  message(STATUS "Found oak_camera_integration: 1.0.0 (${oak_camera_integration_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'oak_camera_integration' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${oak_camera_integration_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(oak_camera_integration_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${oak_camera_integration_DIR}/${_extra}")
endforeach()
