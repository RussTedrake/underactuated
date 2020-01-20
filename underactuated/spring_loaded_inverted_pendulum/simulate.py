
import numpy as np
import matplotlib.pyplot as plt

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource, LogOutput

from underactuated.spring_loaded_inverted_pendulum import (
  SLIPState, SLIPVisualizer, SpringLoadedInvertedPendulum
)

builder = DiagramBuilder()
plant = builder.AddSystem(SpringLoadedInvertedPendulum())

# Parameters from Geyer05, Figure 2.4
# Note: Geyer uses angle of attack = 90 - touchdown_angle
touchdown_angle = np.deg2rad(30)
Etilde = 1.61

s = SLIPState(np.zeros(8))
s.z = 0.9
s.theta = touchdown_angle
s.r = 1
s.xdot = plant.apex_velocity_from_dimensionless_system_energy(Etilde, s.z)

viz = builder.AddSystem(SLIPVisualizer())
builder.Connect(plant.get_output_port(0), viz.get_input_port(0))

log = LogOutput(plant.get_output_port(0), builder)

command = builder.AddSystem(ConstantVectorSource([touchdown_angle]))
builder.Connect(command.get_output_port(0), plant.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
context = simulator.get_mutable_context()
context.SetAccuracy(1e-10)

context.get_mutable_continuous_state_vector().SetFromVector(s[:])

simulator.set_target_realtime_rate(1.0)
simulator.AdvanceTo(0.6)

print("apex: " + str(plant.last_apex))

t = log.sample_times()
x = log.data()
fig, ax = plt.subplots(9)
ax[0].plot(x[0, :], x[1, :])
ax[0].set_xlabel('x')
ax[0].set_ylabel('z')
ax[0].axis('equal')
for i in range(8):
    ax[i+1].plot(t, x[i, :])
    ax[i+1].set_xlabel('t')
    ax[i+1].set_ylabel(s.get_fields()[i])

plt.show()
