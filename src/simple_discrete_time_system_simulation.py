from simple_discrete_time_system import *
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import SignalLogger
from pydrake.systems.analysis import Simulator
import matplotlib.pyplot as plt

# Create a simple block diagram containing our system.
builder = DiagramBuilder()
system = builder.AddSystem(SimpleDiscreteTimeSystem())
# TODO(russt): add binding then replace the next two lines with
#   logger = LogOutput(system->get_output_port(0), builder)
logger = builder.AddSystem(SignalLogger(1))
builder.Connect(system.get_output_port(0), logger.get_input_port(0))
diagram = builder.Build()

# Create the simulator.
simulator = Simulator(diagram)

# Set the initial conditions, x(0).
state = simulator.get_mutable_context().get_mutable_state()\
                 .get_mutable_discrete_state(0).get_mutable_vector()
state.SetFromVector([0.9])

# Simulate for 10 seconds.
simulator.StepTo(10)

# Plot the results.
plt.stem(logger.sample_times(), logger.data().transpose())
plt.xlabel('t')
plt.ylabel('x(t)')
plt.show()