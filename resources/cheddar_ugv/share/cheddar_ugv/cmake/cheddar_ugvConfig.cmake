# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cheddar_ugv_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cheddar_ugv_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cheddar_ugv_FOUND FALSE)
  elseif(NOT cheddar_ugv_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cheddar_ugv_FOUND FALSE)
  endif()
  return()
endif()
set(_cheddar_ugv_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cheddar_ugv_FIND_QUIETLY)
  message(STATUS "Found cheddar_ugv: 0.0.0 (${cheddar_ugv_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cheddar_ugv' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cheddar_ugv_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cheddar_ugv_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cheddar_ugv_DIR}/${_extra}")
endforeach()
