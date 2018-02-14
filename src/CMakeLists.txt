


add_executable(simple_continuous_time_system simple_continuous_time_system.cc)
target_link_libraries(simple_continuous_time_system
    drake::drake)
add_test(NAME simple_continuous_time_system COMMAND simple_continuous_time_system)

find_package(PythonInterp REQUIRED)
find_program(PYCODESTYLE_EXECUTABLE pycodestyle REQUIRED)  # pip install pycodestyle

function(add_pytest pyfile)

  add_test(NAME ${pyfile}
           COMMAND ${PYTHON_EXECUTABLE} ${pyfile} ${ARGN}
           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  set_property(TEST ${pyfile}
               PROPERTY ENVIRONMENT "MPLBACKEND=Template")  # So that pyplot.show() does not block

  add_test(NAME ${pyfile}_pycodestyle
           COMMAND "${PYCODESTYLE_EXECUTABLE}" --count --show-source ${pyfile}
           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

endfunction(add_pytest)

function(add_nbtest ipynbfile)

  add_test(NAME ${ipynbfile}
           COMMAND ${PYTHON_EXECUTABLE} test_ipynb.py ${ipynbfile}
           WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/src)

endfunction(add_nbtest)

add_subdirectory(underactuated)

# Note: no duration arguments to keep this first textbook example simple.
# Just pay the 10second cost.
add_pytest(double_pendulum_simulator.py)

add_nbtest(double_pendulum_simulator.ipynb)

add_pytest(double_pendulum_manipulator.py)
add_pytest(double_pendulum_as_single_pendulum.py --duration 1.0)

add_pytest(pendulum_torque_slider_demo.py --duration 1.0)

add_pytest(simple_continuous_time_system_simulation.py)
add_pytest(simple_discrete_time_system_simulation.py)

add_pytest(double_integrator_value_iteration.py)
add_pytest(pendulum_value_iteration.py)