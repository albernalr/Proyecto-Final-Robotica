# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_phantom_vision_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED phantom_vision_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(phantom_vision_FOUND FALSE)
  elseif(NOT phantom_vision_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(phantom_vision_FOUND FALSE)
  endif()
  return()
endif()
set(_phantom_vision_CONFIG_INCLUDED TRUE)

# output package information
if(NOT phantom_vision_FIND_QUIETLY)
  message(STATUS "Found phantom_vision: 0.0.0 (${phantom_vision_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'phantom_vision' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${phantom_vision_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(phantom_vision_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${phantom_vision_DIR}/${_extra}")
endforeach()
