import argparse
import math
import numpy as np

from pydrake.all import (AddMultibodyPlantSceneGraph, DiagramBuilder,
                         LinearQuadraticRegulator, Parser,
                         PlanarSceneGraphVisualizer, Simulator)
from underactuated import FindResource


def UprightState():
    state = (0, math.pi, 0, 0)
    return state


def BalancingLQR(plant):
    # Design an LQR controller for stabilizing the CartPole around the upright.
    # Returns a (static) AffineSystem that implements the controller (in
    # the original CartPole coordinates).

    context = plant.CreateDefaultContext()
    plant.get_actuation_input_port().FixValue(context, [0])

    context.get_mutable_continuous_state_vector().SetFromVector(UprightState())

    Q = np.diag((10., 10., 1., 1.))
    R = [1]

    # MultibodyPlant has many (optional) input ports, so we must pass the
    # input_port_index to LQR.
    return LinearQuadraticRegulator(
        plant,
        context,
        Q,
        R,
        input_port_index=plant.get_actuation_input_port().get_index())


if __name__ == "__main__":
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    file_name = FindResource("cartpole/cartpole.urdf")
    Parser(plant).AddModelFromFile(file_name)
    plant.Finalize()

    controller = builder.AddSystem(BalancingLQR(plant))
    builder.Connect(plant.get_state_output_port(), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0),
                    plant.get_actuation_input_port())

    visualizer = builder.AddSystem(
        PlanarSceneGraphVisualizer(scene_graph,
                                   xlim=[-2.5, 2.5],
                                   ylim=[-1, 2.5]))
    builder.Connect(scene_graph.get_pose_bundle_output_port(),
                    visualizer.get_input_port(0))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)
    context = simulator.get_mutable_context()

    parser = argparse.ArgumentParser()
    parser.add_argument("-N",
                        "--trials",
                        type=int,
                        help="Number of trials to run.",
                        default=5)
    parser.add_argument("-T",
                        "--duration",
                        type=float,
                        help="Duration to run each sim.",
                        default=10.0)
    args = parser.parse_args()

    for i in range(args.trials):
        context.SetTime(0.)
        context.SetContinuousState(UprightState() + 0.1 * np.random.randn(4,))
        simulator.Initialize()
        simulator.AdvanceTo(args.duration)
