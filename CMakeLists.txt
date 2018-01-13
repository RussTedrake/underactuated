cmake_minimum_required(VERSION 3.5.1)
project(underactuated)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

include(CTest)
include(GNUInstallDirs)

find_package(PythonInterp 2.7 REQUIRED)

find_package(drake CONFIG REQUIRED)

add_test(NAME htmllint WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND tidy -e -config .tidy.config underactuated.html)
set_tests_properties(htmllint PROPERTIES PASS_REGULAR_EXPRESSION " 0 errors")
add_subdirectory(src)
