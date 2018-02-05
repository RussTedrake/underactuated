
add_executable(simple_continuous_time_system simple_continuous_time_system.cc)
target_link_libraries(simple_continuous_time_system
    drake::drake)
add_test(NAME simple_continuous_time_system COMMAND simple_continuous_time_system)

find_package(PythonInterp REQUIRED)

function(add_pytest pyfile)

  add_test(NAME ${pyfile}
           COMMAND ${PYTHON_EXECUTABLE} ${pyfile} ${ARGN}
           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  set_property(TEST ${pyfile}
               PROPERTY ENVIRONMENT "MPLBACKEND=Template")  # So that pyplot.show() does not block

endfunction(add_pytest)

# Note: no duration arguments to keep this first textbook example simple.
# Just pay the 10second cost.
add_pytest(double_pendulum_simulator.py)

add_pytest(double_pendulum_manipulator.py)

add_pytest(pendulum_constant_torque_demo.py 1.0 1.0)

add_pytest(simple_continuous_time_system_simulation.py)
add_pytest(simple_discrete_time_system_simulation.py)

add_pytest(planar_rigid_body_visualizer.py -t 1.0 -T 1.0 -m pend dpend)
