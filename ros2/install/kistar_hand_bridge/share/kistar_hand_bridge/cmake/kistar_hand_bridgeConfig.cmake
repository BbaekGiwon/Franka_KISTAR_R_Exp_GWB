# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_kistar_hand_bridge_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED kistar_hand_bridge_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(kistar_hand_bridge_FOUND FALSE)
  elseif(NOT kistar_hand_bridge_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(kistar_hand_bridge_FOUND FALSE)
  endif()
  return()
endif()
set(_kistar_hand_bridge_CONFIG_INCLUDED TRUE)

# output package information
if(NOT kistar_hand_bridge_FIND_QUIETLY)
  message(STATUS "Found kistar_hand_bridge: 0.1.0 (${kistar_hand_bridge_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'kistar_hand_bridge' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${kistar_hand_bridge_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(kistar_hand_bridge_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${kistar_hand_bridge_DIR}/${_extra}")
endforeach()
