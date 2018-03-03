import argparse
import numpy as np

from pydrake.all import (DiagramBuilder, FloatingBaseType, RigidBodyPlant,
                         RigidBodyTree, Simulator, VectorSystem)
from underactuated import (FindResource, PlanarRigidBodyVisualizer,
                           SliderSystem)


tree = RigidBodyTree(FindResource("acrobot/acrobot.urdf"),
                     FloatingBaseType.kFixed)

builder = DiagramBuilder()
acrobot = builder.AddSystem(RigidBodyPlant(tree))

parser = argparse.ArgumentParser()
parser.add_argument("-T", "--duration",
                    type=float,
                    help="Duration to run sim.",
                    default=10000.0)
args = parser.parse_args()

visualizer = builder.AddSystem(PlanarRigidBodyVisualizer(tree,
                                                         xlim=[-4., 4.],
                                                         ylim=[-4., 4.]))
builder.Connect(acrobot.get_output_port(0), visualizer.get_input_port(0))

ax = visualizer.fig.add_axes([.2, .95, .6, .025])
torque_system = builder.AddSystem(SliderSystem(ax, 'Torque', -5., 5.))
builder.Connect(torque_system.get_output_port(0),
                acrobot.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

state = simulator.get_mutable_context().get_mutable_continuous_state_vector()
state.SetFromVector([1., 0., 0., 0.])

simulator.StepTo(args.duration)
