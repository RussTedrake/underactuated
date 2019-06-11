

# In order to see the visualization, please follow the instructions printed at
# the console to open a web browser.

# Alternatively, you may start the 3D visualizer yourself by running
# `meshcat-server` in your shell, and then opening the url printed to the
# screen in your web browser. Then use `<python> lqr.py --meshcat default` to
# run this script and see that visualization, where `<python>` is either
# `python2` or `python3`.

import argparse
import numpy as np

from pydrake.examples.quadrotor import (QuadrotorPlant,
                                        StabilizingLQRController,
                                        QuadrotorGeometry)
from pydrake.geometry import SceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, VectorSystem
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer

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
scene_graph = builder.AddSystem(SceneGraph())
QuadrotorGeometry.AddToBuilder(builder, plant.get_output_port(0), scene_graph)
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
    context.SetTime(0.)
    context.SetContinuousState(np.random.randn(12,))
    simulator.Initialize()
    simulator.StepTo(args.duration)
