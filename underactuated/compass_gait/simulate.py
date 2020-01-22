import argparse
import math
import numpy as np

from pydrake.all import (ConstantVectorSource,
                         DiagramBuilder,
                         PlanarSceneGraphVisualizer,
                         SceneGraph,
                         SignalLogger,
                         Simulator)
from pydrake.examples.compass_gait import (CompassGait, CompassGaitGeometry,
                                           CompassGaitParams)


builder = DiagramBuilder()
compass_gait = builder.AddSystem(CompassGait())

hip_torque = builder.AddSystem(ConstantVectorSource([0.0]))
builder.Connect(hip_torque.get_output_port(0), compass_gait.get_input_port(0))

parser = argparse.ArgumentParser()
parser.add_argument("-T", "--duration",
                    type=float,
                    help="Duration to run sim.",
                    default=10.0)
args = parser.parse_args()

scene_graph = builder.AddSystem(SceneGraph())
CompassGaitGeometry.AddToBuilder(
    builder, compass_gait.get_floating_base_state_output_port(), scene_graph)
visualizer = builder.AddSystem(PlanarSceneGraphVisualizer(scene_graph,
                                                          xlim=[-1., 5.],
                                                          ylim=[-1., 2.]))
builder.Connect(scene_graph.get_pose_bundle_output_port(),
                visualizer.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)

context = simulator.get_mutable_context()
diagram.Publish(context)  # draw once to get the window open
context.SetAccuracy(1e-4)
context.SetContinuousState([0., 0., 0.4, -2.])

simulator.AdvanceTo(args.duration)
