#!/usr/bin/env python

import sys
import numpy as np

import pydrake.systems.framework
from pydrake.examples.pendulum import PendulumPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource, SignalLogger

from pendulum_visualizer import PendulumVisualizer
import matplotlib.pyplot as plt


builder = DiagramBuilder()
pendulum = builder.AddSystem(PendulumPlant())

torque = 1.0
simulation_duration = 1000
if (len(sys.argv)>1):
    torque = float(sys.argv[1])
if (len(sys.argv)>2):
    simulation_duration = float(sys.argv[2])

torque_system = builder.AddSystem(ConstantVectorSource([torque]))
builder.Connect(torque_system.get_output_port(0),
                        pendulum.get_input_port(0))
print('Simulating with constant torque = ' + str(torque) + ' Newton-meters')

visualizer = builder.AddSystem(PendulumVisualizer())
builder.Connect(pendulum.get_output_port(0), visualizer.get_input_port(0))

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

