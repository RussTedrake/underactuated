import argparse
import numpy as np

from pydrake.all import (AddMultibodyPlantSceneGraph, DiagramBuilder, Parser,
                         PlanarSceneGraphVisualizer, Simulator)
from underactuated import FindResource, SliderSystem

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
file_name = FindResource("cartpole/cartpole.urdf")
Parser(plant).AddModelFromFile(file_name)
plant.Finalize()

parser = argparse.ArgumentParser()
parser.add_argument("-T",
                    "--duration",
                    type=float,
                    help="Duration to run sim.",
                    default=10000.0)
args = parser.parse_args()

visualizer = builder.AddSystem(
    PlanarSceneGraphVisualizer(scene_graph, xlim=[-2.5, 2.5], ylim=[-1, 2.5]))
builder.Connect(scene_graph.get_pose_bundle_output_port(),
                visualizer.get_input_port(0))

ax = visualizer.fig.add_axes([.2, .95, .6, .025])
torque_system = builder.AddSystem(SliderSystem(ax, "Force", -30., 30.))
builder.Connect(torque_system.get_output_port(0),
                plant.get_actuation_input_port())

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

context = simulator.get_mutable_context()
context.SetContinuousState([0., 1., 0., 0.])

simulator.AdvanceTo(args.duration)
