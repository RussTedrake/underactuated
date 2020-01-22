import argparse
import math
import numpy as np

from pydrake.all import (DiagramBuilder, PlanarSceneGraphVisualizer,
                         SceneGraph, Simulator)
from pydrake.examples.rimless_wheel import (RimlessWheel, RimlessWheelGeometry,
                                            RimlessWheelParams)


parser = argparse.ArgumentParser()
parser.add_argument("-T", "--duration",
                    type=float,
                    help="Duration to run sim.",
                    default=10.0)
parser.add_argument("-Q", "--initial_angle",
                    type=float,
                    help="Initial angle of the stance leg (in radians).",
                    default=0.0)
parser.add_argument("-V", "--initial_angular_velocity",
                    type=float,
                    help="Initial angular velocity of the stance leg "
                         "(in radians/sec).",
                    default=5.0)
parser.add_argument("-S", "--slope", type=float,
                    help="Ramp angle (in radians)",
                    default=0.08)
args = parser.parse_args()

params = RimlessWheelParams()
params.set_slope(args.slope)

builder = DiagramBuilder()
rimless_wheel = builder.AddSystem(RimlessWheel())
scene_graph = builder.AddSystem(SceneGraph())
RimlessWheelGeometry.AddToBuilder(
    builder, rimless_wheel.get_floating_base_state_output_port(), params,
    scene_graph)
visualizer = builder.AddSystem(PlanarSceneGraphVisualizer(scene_graph,
                                                          xlim=[-8., 8.],
                                                          ylim=[-2., 3.]))
builder.Connect(scene_graph.get_pose_bundle_output_port(),
                visualizer.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)

context = simulator.get_mutable_context()
diagram.Publish(context)  # draw once to get the window open
diagram.GetMutableSubsystemContext(
    rimless_wheel, context).get_numeric_parameter(0).set_slope(args.slope)
context.SetAccuracy(1e-4)
context.SetContinuousState([args.initial_angle, args.initial_angular_velocity])

simulator.AdvanceTo(args.duration)
