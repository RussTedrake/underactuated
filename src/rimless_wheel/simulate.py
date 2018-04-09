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


tree = RigidBodyTree(FindResourceOrThrow(
                        "drake/examples/rimless_wheel/RimlessWheel.urdf"),
                     FloatingBaseType.kRollPitchYaw)
params = RimlessWheelParams()
R = np.identity(3)
R[0, 0] = math.cos(params.slope())
R[0, 2] = math.sin(params.slope())
R[2, 0] = -math.sin(params.slope())
R[2, 2] = math.cos(params.slope())
T = Isometry3(rotation=R, translation=[0, 0, -5.])
color = np.array([0.9297, 0.7930, 0.6758, 1])
tree.world().AddVisualElement(VisualElement(Box([100., 1., 10.]), T, color))
tree.compile()

builder = DiagramBuilder()
rimless_wheel = builder.AddSystem(RimlessWheel())

parser = argparse.ArgumentParser()
parser.add_argument("-T", "--duration",
                    type=float,
                    help="Duration to run sim.",
                    default=20.0)
parser.add_argument("-Q", "--initial_angle",
                    type=float,
                    help="Initial angle of the stance leg (in radians).",
                    default=0.0)
parser.add_argument("-V", "--initial_angular_velocity",
                    type=float,
                    help="Initial angular velocity of the stance leg "
                         "(in radians/sec).",
                    default=5.0)
args = parser.parse_args()

visualizer = builder.AddSystem(PlanarRigidBodyVisualizer(tree,
                                                         xlim=[-8., 8.],
                                                         ylim=[-4., 4.]))
builder.Connect(rimless_wheel.get_output_port(1), visualizer.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)
simulator.get_mutable_context().set_accuracy(1e-4)

state = simulator.get_mutable_context().get_mutable_continuous_state_vector()
state.SetFromVector([args.initial_angle, args.initial_angular_velocity])

simulator.StepTo(args.duration)
