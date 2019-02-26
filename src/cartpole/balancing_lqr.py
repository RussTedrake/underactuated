import argparse
import math
import numpy as np

from pydrake.all import (BasicVector, DiagramBuilder, FloatingBaseType,
                         LinearQuadraticRegulator, RigidBodyPlant,
                         RigidBodyTree, Simulator)
from underactuated import (FindResource, PlanarRigidBodyVisualizer)


def UprightState():
    state = (0, math.pi, 0, 0)
    return state


def BalancingLQR(robot):
    # Design an LQR controller for stabilizing the CartPole around the upright.
    # Returns a (static) AffineSystem that implements the controller (in
    # the original CartPole coordinates).

    context = robot.CreateDefaultContext()
    context.FixInputPort(0, BasicVector([0]))

    context.get_mutable_continuous_state_vector().SetFromVector(UprightState())

    Q = np.diag((10., 10., 1., 1.))
    R = [1]

    return LinearQuadraticRegulator(robot, context, Q, R)


if __name__ == "__main__":
    builder = DiagramBuilder()

    tree = RigidBodyTree(FindResource("cartpole/cartpole.urdf"),
                         FloatingBaseType.kFixed)

    robot = builder.AddSystem(RigidBodyPlant(tree))
    controller = builder.AddSystem(BalancingLQR(robot))
    builder.Connect(robot.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), robot.get_input_port(0))

    visualizer = builder.AddSystem(PlanarRigidBodyVisualizer(tree,
                                                             xlim=[-2.5, 2.5],
                                                             ylim=[-1, 2.5]))
    builder.Connect(robot.get_output_port(0), visualizer.get_input_port(0))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)
    context = simulator.get_mutable_context()

    parser = argparse.ArgumentParser()
    parser.add_argument("-N", "--trials",
                        type=int,
                        help="Number of trials to run.",
                        default=5)
    parser.add_argument("-T", "--duration",
                        type=float,
                        help="Duration to run each sim.",
                        default=10.0)
    args = parser.parse_args()

    for i in range(args.trials):
        context.set_time(0.)
        context.SetContinuousState(UprightState() + 0.1*np.random.randn(4,))
        simulator.Initialize()
        simulator.StepTo(args.duration)
