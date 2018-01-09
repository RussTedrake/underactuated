cmake_minimum_required(VERSION 3.5)
project(underactuated LANGUAGES NONE)

if(CMAKE_VERSION VERSION_LESS 3.7)
  set(CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/3.7"
    ${CMAKE_MODULE_PATH})
endif()

include(CTest)

/// TODO(russt): find drake (what is the new cmake way to do this?)

add_subdirectory(src)


