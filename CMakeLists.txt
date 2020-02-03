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

find_package(PythonInterp REQUIRED)
find_program(PYCODESTYLE_EXECUTABLE pycodestyle REQUIRED)  # pip install pycodestyle

function(add_pystyle_test pyfile)

  get_filename_component(pydir ${CMAKE_CURRENT_SOURCE_DIR} NAME)

  add_test(NAME ${pydir}_${pyfile}_pycodestyle
           COMMAND "${PYCODESTYLE_EXECUTABLE}" --count --show-source ${pyfile}
           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

endfunction(add_pystyle_test)

function(add_pytest pyfile)

  get_filename_component(pydir ${CMAKE_CURRENT_SOURCE_DIR} NAME)

  add_test(NAME ${pydir}_${pyfile}
           COMMAND ${PYTHON_EXECUTABLE} ${pyfile} ${ARGN}
           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  set_property(TEST ${pydir}_${pyfile}
               PROPERTY ENVIRONMENT
                 "MPLBACKEND=Template"  # So that pyplot.show() does not block
                 "PATH=${PATH}"
                 "PYTHONPATH=${PYTHONPATH}"
              )

  set_tests_properties(${pydir}_${pyfile} PROPERTIES
            FAIL_REGULAR_EXPRESSION "DrakeDeprecationWarning"
          )

  add_pystyle_test(${pyfile})

endfunction(add_pytest)

function(add_nbtest ipynbfile)

  get_filename_component(pydir ${CMAKE_CURRENT_SOURCE_DIR} NAME)

  add_test(NAME ${pydir}_${ipynbfile}
           COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/test_ipynb.py ${ipynbfile}
           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  set_property(TEST ${pydir}_${ipynbfile}
               PROPERTY ENVIRONMENT
                 "PYTHONPATH=${PYTHONPATH}"
              )

  set_tests_properties(${pydir}_${ipynbfile} PROPERTIES
          FAIL_REGULAR_EXPRESSION "DrakeDeprecationWarning"
          )

endfunction(add_nbtest)

add_subdirectory(underactuated)
add_subdirectory(examples/double_pendulum)
