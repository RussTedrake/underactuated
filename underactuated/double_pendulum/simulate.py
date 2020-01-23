from pydrake.all import (AddMultibodyPlantSceneGraph, DiagramBuilder, Parser,
                         PlanarSceneGraphVisualizer, Simulator)
from underactuated import FindResource

# Set up a block diagram with the robot (dynamics) and a visualization block.
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

# Load the double pendulum from Universal Robot Description Format
parser = Parser(plant, scene_graph)
parser.AddModelFromFile(FindResource("double_pendulum/double_pendulum.urdf"))
plant.Finalize()

builder.ExportInput(plant.get_actuation_input_port())
visualizer = builder.AddSystem(
    PlanarSceneGraphVisualizer(scene_graph, xlim=[-2.8, 2.8], ylim=[-2.8, 2.8]))
builder.Connect(scene_graph.get_pose_bundle_output_port(),
                visualizer.get_input_port(0))
diagram = builder.Build()

# Set up a simulator to run this diagram
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)

# Set the initial conditions
context = simulator.get_mutable_context()
# state is (theta1, theta2, theta1dot, theta2dot)
context.SetContinuousState([1., 1., 0., 0.])
context.FixInputPort(0, [0., 0.])  # Zero input torques

# Simulate for 10 seconds
simulator.AdvanceTo(10)
