# -*- mode: cmake -*-
# vi: set ft=cmake :

# Copyright (c) 2018, Massachusetts Institute of Technology.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the Massachusetts Institute of Technology nor the names
#   of its contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE MASSACHUSETTS INSTITUTE OF TECHNOLOGY AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
# NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE MASSACHUSETTS
# INSTITUTE OF TECHNOLOGY OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

function(_gflags_find_library _NAME)
  string(TOUPPER ${_NAME} _NAME_UPPER)

  set(_LIBRARY_VAR GFLAGS_${_NAME_UPPER}_LIBRARY)
  set(_TARGET gflags_${_NAME})

  if(_NAME MATCHES static$)
    set(_SHARED_STATIC STATIC)
  else()
    set(_SHARED_STATIC SHARED)
  endif()

  if(_NAME MATCHES ^nothreads)
    set(_NOTHREADS _nothreads)
  else()
    set(_NOTHREADS)
  endif()

  find_library(${_LIBRARY_VAR}
    NAMES ${CMAKE_${_SHARED_STATIC}_LIBRARY_PREFIX}gflags${_NOTHREADS}${CMAKE_${_SHARED_STATIC}_LIBRARY_SUFFIX}
    HINTS "${PC_GFLAGS_LIBRARY_DIRS}"
  )

  set(GFlags_${_NAME}_FIND_QUIETLY ON)
  find_package_handle_standard_args(GFlags_${_NAME}
    REQUIRED_VARS GFLAGS_INCLUDE_DIR ${_LIBRARY_VAR}
  )

  if(GFlags_${_NAME}_FOUND)
    set(GFlags_${_NAME}_FOUND ON PARENT_SCOPE)
    mark_as_advanced(${_LIBRARY_VAR})

    add_library(${_TARGET} ${_SHARED_STATIC} IMPORTED)
    set_target_properties(${_TARGET} PROPERTIES
      IMPORTED_LOCATION "${${_LIBRARY_VAR}}"
      INTERFACE_INCLUDE_DIRECTORIES "${GFLAGS_INCLUDE_DIRS}"
    )

    if(_NAME MATCHES static$)
      set_target_properties(${_TARGET} PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES CXX
        INTERFACE_COMPILE_DEFINITIONS GFLAGS_IS_A_DLL=0
      )
    else()
      if(WIN32)
        set_target_properties(${_TARGET} PROPERTIES
          INTERFACE_COMPILE_DEFINITIONS GFLAGS_IS_A_DLL=1
        )
      else()
        set_target_properties(${_TARGET} PROPERTIES
          INTERFACE_COMPILE_DEFINITIONS GFLAGS_IS_A_DLL=0
        )
      endif()
    endif()

    if(_NAME STREQUAL static)
      find_package(Threads QUIET)

      if(Threads_FOUND)
        set_target_properties(${_TARGET} PROPERTIES
          INTERFACE_LINK_LIBRARIES $<LINK_ONLY:Threads::Threads>
        )
      endif()
    endif()
  endif()
endfunction()

if(NOT GFlags_FIND_COMPONENTS)
  set(GFlags_FIND_COMPONENTS nothreads_shared)
  set(GFlags_FIND_REQUIRED_nothreads_shared ${GFlags_FIND_REQUIRED})
endif()

find_package(PkgConfig QUIET)

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)
pkg_check_modules(PC_GFLAGS QUIET gflags)

if(GFLAGS_VERSION_STRING)
  set(GFLAGS_VERSION_STRING "${PC_GFLAGS_VERSION}")
else()
  set(GFLAGS_VERSION_STRING)
endif()

find_path(GFLAGS_INCLUDE_DIR NAMES gflags/gflags.h
  HINTS "${PC_GFLAGS_INCLUDE_DIRS}"
)

foreach(_GFLAGS_COMPONENT ${GFlags_FIND_COMPONENTS})
  _gflags_find_library(${_GFLAGS_COMPONENT})
endforeach()

if(GFLAGS_NOTHREADS_SHARED_LIBRARY)
  set(GFLAGS_LIBRARY gflags_nothreads_shared)
elseif(GFLAGS_NOTHREADS_STATIC_LIBRARY)
  set(GFLAGS_LIBRARY gflags_nothreads_static)
elseif(GFLAGS_SHARED_LIBRARY)
  set(GFLAGS_LIBRARY gflags_shared)
elseif(GFLAGS_STATIC_LIBRARY)
  set(GFLAGS_LIBRARY gflags_static)
else()
  set(GFLAGS_LIBRARY)
endif()

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(GFlags
  REQUIRED_VARS GFLAGS_INCLUDE_DIR GFLAGS_LIBRARY
  VERSION_VAR GFLAGS_VERSION_STRING
  HANDLE_COMPONENTS
)

if(GFLAGS_FOUND)
  set(GFLAGS_INCLUDE_DIRS "${GFLAGS_INCLUDE_DIR}")
  set(GFLAGS_LIBRARIES "${GFLAGS_LIBRARY}")
  mark_as_advanced(GFLAGS_INCLUDE_DIR)
endif()
