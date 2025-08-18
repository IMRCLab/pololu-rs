# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mocap_to_robots_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mocap_to_robots_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mocap_to_robots_FOUND FALSE)
  elseif(NOT mocap_to_robots_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mocap_to_robots_FOUND FALSE)
  endif()
  return()
endif()
set(_mocap_to_robots_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mocap_to_robots_FIND_QUIETLY)
  message(STATUS "Found mocap_to_robots: 0.0.1 (${mocap_to_robots_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mocap_to_robots' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mocap_to_robots_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mocap_to_robots_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mocap_to_robots_DIR}/${_extra}")
endforeach()
