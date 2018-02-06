#!/usr/bin/env python

import sys
import numpy as np

import pydrake.systems.framework
from pydrake.examples.pendulum import PendulumPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, VectorSystem
from pydrake.systems.primitives import ConstantVectorSource, SignalLogger

from pendulum_visualizer import PendulumVisualizer
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

class TorqueSlider(VectorSystem):
    def __init__(self, ax):
        # 0 inputs, 1 output.
        VectorSystem.__init__(self, 0, 1)
        self.value = 0
        self.slider = Slider(ax, 'Torque', -5, 5, valinit=self.value)
        self.slider.on_changed(self.update)

    def update(self,val):
        self.value = val

    def _DoCalcVectorOutput(self, context, unused, unused2, torque):
        torque[:] = self.value


builder = DiagramBuilder()
pendulum = builder.AddSystem(PendulumPlant())

simulation_duration = 1000
if (len(sys.argv)>2):
    simulation_duration = float(sys.argv[2])

visualizer = builder.AddSystem(PendulumVisualizer())
builder.Connect(pendulum.get_output_port(0), visualizer.get_input_port(0))

ax = visualizer.fig.add_axes([.2, .95, .6, .025])
torque_system = builder.AddSystem(TorqueSlider(ax))
builder.Connect(torque_system.get_output_port(0),
                        pendulum.get_input_port(0))

signalLogRate = 60
signalLogger = builder.AddSystem(SignalLogger(2))
signalLogger._DeclarePeriodicPublish(1. / signalLogRate, 0.0)
builder.Connect(pendulum.get_output_port(0), signalLogger.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.Initialize()
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

state = simulator.get_mutable_context().get_continuous_state_vector()

initial_state = np.array([1.0, 0.0])
state.SetFromVector(initial_state)

simulator.StepTo(simulation_duration)

print(state.CopyToVector())