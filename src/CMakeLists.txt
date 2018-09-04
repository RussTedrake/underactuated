
find_package(PythonInterp REQUIRED)
find_program(PYCODESTYLE_EXECUTABLE pycodestyle REQUIRED)  # pip install pycodestyle

function(add_pytest pyfile)

  get_filename_component(pydir ${CMAKE_CURRENT_SOURCE_DIR} NAME)

  add_test(NAME ${pydir}_${pyfile}
           COMMAND ${PYTHON_EXECUTABLE} ${pyfile} ${ARGN}
           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  set_property(TEST ${pydir}_${pyfile}
               PROPERTY ENVIRONMENT
                 "MPLBACKEND=Template"  # So that pyplot.show() does not block
                 "PYTHONPATH=${PYTHONPATH}"
              )

  add_test(NAME ${pydir}_${pyfile}_pycodestyle
           COMMAND "${PYCODESTYLE_EXECUTABLE}" --count --show-source ${pyfile}
           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

endfunction(add_pytest)

function(add_nbtest ipynbfile)

  get_filename_component(pydir ${CMAKE_CURRENT_SOURCE_DIR} NAME)

  add_test(NAME ${pydir}_${ipynbfile}
           COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/src/test_ipynb.py ${ipynbfile}
           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

endfunction(add_nbtest)

add_subdirectory(underactuated)

add_subdirectory(acrobot)
add_subdirectory(cartpole)
add_subdirectory(compass_gait)
add_subdirectory(double_integrator)
add_subdirectory(double_pendulum)
add_subdirectory(lyapunov)
add_subdirectory(pendulum)
add_subdirectory(rimless_wheel)
add_subdirectory(simple)
add_subdirectory(van_der_pol)