
add_executable(simple_continuous_time_system simple_continuous_time_system.cc)
target_link_libraries(simple_continuous_time_system
    drake::drake)
add_test(NAME simple_continuous_time_system COMMAND simple_continuous_time_system)

find_package(PythonInterp REQUIRED)

add_test(NAME double_pendulum COMMAND ${PYTHON_EXECUTABLE} double_pendulum.py WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
