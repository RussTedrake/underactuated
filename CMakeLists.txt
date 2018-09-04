cmake_minimum_required(VERSION 3.5.1)
project(underactuated)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

include(CTest)
include(GNUInstallDirs)

find_package(PythonInterp 2.7 REQUIRED)

find_package(drake CONFIG REQUIRED)

set(PYTHONPATH "${CMAKE_CURRENT_SOURCE_DIR}/src:${drake_PYTHON_DIRS}:$ENV{PYTHONPATH}")

add_test(NAME htmllint WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND tidy -e -config .tidy.config --show-warnings no underactuated.html)
set_tests_properties(htmllint PROPERTIES PASS_REGULAR_EXPRESSION " 0 errors")
add_subdirectory(src)
