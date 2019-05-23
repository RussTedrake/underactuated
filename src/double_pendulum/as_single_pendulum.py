# -*- coding: utf-8 -*-

import argparse
from math import sin

from pydrake.all import (AddMultibodyPlantSceneGraph,
                         DiagramBuilder,
                         Parser,
                         Simulator,
                         VectorSystem)
from underactuated import (FindResource,
                           ManipulatorDynamics,
                           PlanarSceneGraphVisualizer)


class Controller(VectorSystem):
    """ Defines a feedback controller for the double pendulum.

    The controller applies torques at the joints in order to
    1) cancel out the dynamics of the double pendulum,
    2) make the first joint swing with the dynamics of a single pendulum, and
    3) drive the second joint towards zero.

    The magnitude of gravity for the imposed single pendulum dynamics is taken
    as a constructor argument.  So you can do fun things like pretending that
    gravity is zero, or even inverting gravity!

    """

    def __init__(self, multibody_plant, gravity):
        # 4 inputs (double pend state), 2 torque outputs.
        VectorSystem.__init__(self, 4, 2)
        self.plant = multibody_plant
        self.g = gravity

    def DoCalcVectorOutput(self, context, double_pend_state, unused, torque):
        # Extract manipulator dynamics.
        q = double_pend_state[:2]
        v = double_pend_state[-2:]
        (M, Cv, tauG, B, tauExt) = ManipulatorDynamics(self.plant, q, v)

        # Desired pendulum parameters.
        length = 2.
        b = .1

        # Control gains for stabilizing the second joint.
        kp = 1
        kd = .1

        # Cancel double pend dynamics and inject single pend dynamics.
        torque[:] = Cv - tauG - tauExt + \
            M.dot([-self.g / length * sin(q[0]) - b * v[0],
                   -kp * q[1] - kd * v[1]])


parser = argparse.ArgumentParser()
parser.add_argument("-g", "--gravity",
                    type=float,
                    help="Gravitational constant (m/s^2).",
                    default=9.8)
parser.add_argument("-T", "--duration",
                    type=float,
                    help="Duration to run sim.",
                    default=10.0)
args = parser.parse_args()

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder)

# Load the double pendulum from Universal Robot Description Format
parser = Parser(plant, scene_graph)
parser.AddModelFromFile(FindResource("double_pendulum/double_pendulum.urdf"))
plant.Finalize()

controller = builder.AddSystem(Controller(plant, args.gravity))
builder.Connect(plant.get_state_output_port(),
                controller.get_input_port(0))
builder.Connect(controller.get_output_port(0),
                plant.get_actuation_input_port())

visualizer = builder.AddSystem(PlanarSceneGraphVisualizer(scene_graph,
                                                          xlim=[-2.8, 2.8],
                                                          ylim=[-2.8, 2.8]))
builder.Connect(scene_graph.get_pose_bundle_output_port(),
                visualizer.get_input_port(0))

diagram = builder.Build()

# Set up a simulator to run this diagram
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)

# Set the initial conditions
context = simulator.get_mutable_context()
context.SetContinuousState((1., 0., 0.2, 0.))  # (θ₁, θ₂, θ̇₁, θ̇₂)

# Simulate
simulator.StepTo(args.duration)
