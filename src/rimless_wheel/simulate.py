import argparse
import math
import numpy as np

from pydrake.all import (Box,
                         DiagramBuilder,
                         FindResourceOrThrow,
                         FloatingBaseType,
                         Isometry3,
                         RigidBodyTree,
                         Simulator,
                         VisualElement)
from pydrake.examples.rimless_wheel import (RimlessWheel, RimlessWheelParams)
from underactuated import (PlanarRigidBodyVisualizer)

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

tree = RigidBodyTree(FindResourceOrThrow(
                        "drake/examples/rimless_wheel/RimlessWheel.urdf"),
                     FloatingBaseType.kRollPitchYaw)
params = RimlessWheelParams()
params.set_slope(args.slope)
R = np.identity(3)
R[0, 0] = math.cos(params.slope())
R[0, 2] = math.sin(params.slope())
R[2, 0] = -math.sin(params.slope())
R[2, 2] = math.cos(params.slope())
X = Isometry3(rotation=R, translation=[0, 0, -5.])
color = np.array([0.9297, 0.7930, 0.6758, 1])
tree.world().AddVisualElement(VisualElement(Box([100., 1., 10.]), X, color))
tree.compile()

builder = DiagramBuilder()
rimless_wheel = builder.AddSystem(RimlessWheel())

visualizer = builder.AddSystem(PlanarRigidBodyVisualizer(tree,
                                                         xlim=[-8., 8.],
                                                         ylim=[-2., 3.],
                                                         figsize_multiplier=3))
builder.Connect(rimless_wheel.get_output_port(1), visualizer.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)

context = simulator.get_mutable_context()
diagram.Publish(context)  # draw once to get the window open
diagram.GetMutableSubsystemContext(
    rimless_wheel, context).get_numeric_parameter(0).set_slope(args.slope)
context.set_accuracy(1e-4)
context.SetContinuousState([args.initial_angle, args.initial_angular_velocity])

simulator.StepTo(args.duration)
