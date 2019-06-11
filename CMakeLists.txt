cmake_minimum_required(VERSION 3.5.1)
project(underactuated)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

include(CTest)
include(GNUInstallDirs)

if(APPLE)
    set(PYTHON_VERSION_MAJOR_MINOR 3.7)
elseif(UNIX)
    set(UNIX_DISTRIBUTION_CODENAME)
    execute_process(COMMAND lsb_release --codename --short
        RESULT_VARIABLE _RETURN_CODE
        OUTPUT_VARIABLE UNIX_DISTRIBUTION_CODENAME
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT _RETURN_CODE EQUAL 0)
        message(FATAL_ERROR "Could not run 'lsb_release'")
    endif()
    if(UNIX_DISTRIBUTION_CODENAME STREQUAL "xenial")
        set(PYTHON_VERSION_MAJOR_MINOR 2.7)
    elseif(UNIX_DISTRIBUTION_CODENAME STREQUAL "bionic")
        set(PYTHON_VERSION_MAJOR_MINOR 3.6)
    else()
        message(FATAL_ERROR "Unsupported linux platform: ${UNIX_DISTRIBUTION_CODENAME}")
    endif()
else()
    message(FATAL_ERROR "Unsupported platform")
endif()

find_package(PythonInterp ${PYTHON_VERSION_MAJOR_MINOR} REQUIRED)

find_package(drake CONFIG REQUIRED)

set(PYTHONPATH "${CMAKE_CURRENT_SOURCE_DIR}/src:${drake_PYTHON_DIRS}:$ENV{PYTHONPATH}")

add_test(NAME htmllint WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND tidy -e -config .tidy.config --show-warnings no underactuated.html)
set_tests_properties(htmllint PROPERTIES PASS_REGULAR_EXPRESSION " 0 errors")
add_subdirectory(src)
