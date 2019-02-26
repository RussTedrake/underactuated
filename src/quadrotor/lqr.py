
import argparse
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.examples.quadrotor import QuadrotorPlant, StabilizingLQRController
from pydrake.geometry import SceneGraph
from pydrake.math import RollPitchYaw
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, VectorSystem
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.systems.rendering import MultibodyPositionToGeometryPose
from pydrake.systems.primitives import AffineSystem

parser = argparse.ArgumentParser()
parser.add_argument("-N", "--trials",
                    type=int,
                    help="Number of trials to run.",
                    default=5)
parser.add_argument("-T", "--duration",
                    type=float,
                    help="Duration to run each sim.",
                    default=4.0)
MeshcatVisualizer.add_argparse_argument(parser)
args = parser.parse_args()


builder = DiagramBuilder()

plant = builder.AddSystem(QuadrotorPlant())

controller = builder.AddSystem(StabilizingLQRController(plant, [0, 0, 1]))
builder.Connect(controller.get_output_port(0), plant.get_input_port(0))
builder.Connect(plant.get_output_port(0), controller.get_input_port(0))

# Set up visualization in MeshCat
# TODO(russt): simplify this (by teaching QuadrotorPlant about SceneGraph)
# and/or hide the details.


class QuadrotorStateToPose(VectorSystem):
    def __init__(self):
        VectorSystem.__init__(self, 12, 7)

    def _DoCalcVectorOutput(self, context, u, x, y):
        y[:] = np.hstack((RollPitchYaw(u[3:6]).ToQuaternion().wxyz(), u[:3]))


scene_graph = builder.AddSystem(SceneGraph())
mbp = MultibodyPlant()
mbp.RegisterAsSourceForSceneGraph(scene_graph)
parser = Parser(mbp)
parser.AddModelFromFile(FindResourceOrThrow(
    "drake/examples/quadrotor/quadrotor.urdf"))
mbp.Finalize(scene_graph)
to_mbp_pose = builder.AddSystem(QuadrotorStateToPose())
builder.Connect(plant.get_output_port(0), to_mbp_pose.get_input_port(0))
to_geom_pose = builder.AddSystem(MultibodyPositionToGeometryPose(mbp))
builder.Connect(to_mbp_pose.get_output_port(0), to_geom_pose.get_input_port())
builder.Connect(
    to_geom_pose.get_output_port(),
    scene_graph.get_source_pose_port(mbp.get_source_id()))
meshcat = builder.AddSystem(MeshcatVisualizer(
    scene_graph, zmq_url=args.meshcat,
    open_browser=args.open_browser))
builder.Connect(scene_graph.get_pose_bundle_output_port(),
                meshcat.get_input_port(0))
# end setup for visualization

diagram = builder.Build()

simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
context = simulator.get_mutable_context()

for i in range(args.trials):
    context.set_time(0.)
    context.SetContinuousState(np.random.randn(12,))
    simulator.Initialize()
    simulator.StepTo(args.duration)
