cmake_minimum_required(VERSION 3.10)
project(underactuated)

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
    if(UNIX_DISTRIBUTION_CODENAME STREQUAL "bionic")
        set(PYTHON_VERSION_MAJOR_MINOR 3.6)
    else()
        message(FATAL_ERROR "Unsupported linux platform: ${UNIX_DISTRIBUTION_CODENAME}")
    endif()
else()
    message(FATAL_ERROR "Unsupported platform")
endif()

find_package(PythonInterp ${PYTHON_VERSION_MAJOR_MINOR} REQUIRED)

find_package(drake CONFIG REQUIRED)
get_filename_component(DRAKE_ROOT "${drake_DIR}/../../../" REALPATH)

set(PATH "${DRAKE_ROOT}/bin:$ENV{PATH}")
set(PYTHONPATH "${CMAKE_CURRENT_SOURCE_DIR}:${DRAKE_ROOT}/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages:$ENV{PYTHONPATH}")

find_program(TIDY_EXECUTABLE NAMES tidy)
if(NOT TIDY_EXECUTABLE)
  message(FATAL_ERROR "tidy could NOT be found")
endif()

add_test(NAME htmllint WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND "${TIDY_EXECUTABLE}" -e -config .tidy.config --show-warnings no underactuated.html)
set_tests_properties(htmllint PROPERTIES PASS_REGULAR_EXPRESSION " 0 errors")
add_subdirectory(underactuated)
