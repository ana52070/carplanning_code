# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_carplanning_code_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED carplanning_code_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(carplanning_code_FOUND FALSE)
  elseif(NOT carplanning_code_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(carplanning_code_FOUND FALSE)
  endif()
  return()
endif()
set(_carplanning_code_CONFIG_INCLUDED TRUE)

# output package information
if(NOT carplanning_code_FIND_QUIETLY)
  message(STATUS "Found carplanning_code: 0.1.0 (${carplanning_code_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'carplanning_code' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${carplanning_code_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(carplanning_code_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${carplanning_code_DIR}/${_extra}")
endforeach()
