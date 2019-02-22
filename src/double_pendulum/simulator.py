from pydrake.all import (DiagramBuilder, FloatingBaseType,
                         RigidBodyPlant, RigidBodyTree, Simulator)
from underactuated import (FindResource, PlanarRigidBodyVisualizer)

# Load the double pendulum from Universal Robot Description Format
tree = RigidBodyTree(FindResource("double_pendulum/double_pendulum.urdf"),
                     FloatingBaseType.kFixed)

# Set up a block diagram with the robot (dynamics) and a visualization block.
builder = DiagramBuilder()
robot = builder.AddSystem(RigidBodyPlant(tree))
builder.ExportInput(robot.get_input_port(0))
visualizer = builder.AddSystem(PlanarRigidBodyVisualizer(tree,
                                                         xlim=[-2.8, 2.8],
                                                         ylim=[-2.8, 2.8]))
builder.Connect(robot.get_output_port(0), visualizer.get_input_port(0))
diagram = builder.Build()

# Set up a simulator to run this diagram
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

# Set the initial conditions
context = simulator.get_mutable_context()
# state is (theta1, theta2, theta1dot, theta2dot)
context.SetContinuousState([1., 1., 0., 0.])
context.FixInputPort(0, [0., 0.])  # Zero input torques

# Simulate for 10 seconds
simulator.StepTo(10)
