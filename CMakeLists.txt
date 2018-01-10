cmake_minimum_required(VERSION 3.5.1)
project(underactuated)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

include(CTest)
include(GNUInstallDirs)

find_package(PythonInterp 2.7 REQUIRED)

find_package(drake CONFIG REQUIRED)

add_subdirectory(src)
