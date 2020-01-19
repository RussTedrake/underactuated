import argparse
import numpy as np

from pydrake.all import (DiagramBuilder, PlanarSceneGraphVisualizer, Simulator,
                         SceneGraph, VectorSystem)
from pydrake.examples.acrobot import (AcrobotGeometry, AcrobotInput,
                                      AcrobotPlant, AcrobotState)
from underactuated import SliderSystem


parser = argparse.ArgumentParser()
parser.add_argument("-T", "--duration",
                    type=float,
                    help="Duration to run sim.",
                    default=10000.0)
args = parser.parse_args()

builder = DiagramBuilder()
acrobot = builder.AddSystem(AcrobotPlant())

scene_graph = builder.AddSystem(SceneGraph())
AcrobotGeometry.AddToBuilder(builder, acrobot.get_output_port(0),
                             scene_graph)
visualizer = builder.AddSystem(PlanarSceneGraphVisualizer(scene_graph,
                                                          xlim=[-4., 4.],
                                                          ylim=[-4., 4.]))
builder.Connect(scene_graph.get_pose_bundle_output_port(),
                visualizer.get_input_port(0))

ax = visualizer.fig.add_axes([.2, .95, .6, .025])
torque_system = builder.AddSystem(SliderSystem(ax, 'Torque', -5., 5.))
builder.Connect(torque_system.get_output_port(0),
                acrobot.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

context = simulator.get_mutable_context()
context.SetContinuousState([1., 0., 0., 0.])

simulator.AdvanceTo(args.duration)
