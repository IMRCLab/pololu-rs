# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_crazyflie_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED crazyflie_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(crazyflie_FOUND FALSE)
  elseif(NOT crazyflie_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(crazyflie_FOUND FALSE)
  endif()
  return()
endif()
set(_crazyflie_CONFIG_INCLUDED TRUE)

# output package information
if(NOT crazyflie_FIND_QUIETLY)
  message(STATUS "Found crazyflie: 1.0.3 (${crazyflie_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'crazyflie' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${crazyflie_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(crazyflie_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${crazyflie_DIR}/${_extra}")
endforeach()
