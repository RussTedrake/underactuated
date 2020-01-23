import math
import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (DiagramBuilder, DirectCollocation, MultibodyPlant,
                         MultibodyPositionToGeometryPose, Parser,
                         PiecewisePolynomial, PlanarSceneGraphVisualizer,
                         SceneGraph, Simulator, Solve, TrajectorySource)
from underactuated import FindResource

plant = MultibodyPlant(time_step=0.0)
scene_graph = SceneGraph()
plant.RegisterAsSourceForSceneGraph(scene_graph)
file_name = FindResource("cartpole/cartpole.urdf")
Parser(plant).AddModelFromFile(file_name)
plant.Finalize()

context = plant.CreateDefaultContext()
dircol = DirectCollocation(
    plant,
    context,
    num_time_samples=21,
    minimum_timestep=0.1,
    maximum_timestep=0.4,
    input_port_index=plant.get_actuation_input_port().get_index())

dircol.AddEqualTimeIntervalsConstraints()

initial_state = (0., 0., 0., 0.)
dircol.AddBoundingBoxConstraint(initial_state, initial_state,
                                dircol.initial_state())
# More elegant version is blocked by drake #8315:
# dircol.AddLinearConstraint(dircol.initial_state() == initial_state)

final_state = (0., math.pi, 0., 0.)
dircol.AddBoundingBoxConstraint(final_state, final_state, dircol.final_state())
# dircol.AddLinearConstraint(dircol.final_state() == final_state)

R = 10  # Cost on input "effort".
u = dircol.input()
dircol.AddRunningCost(R * u[0]**2)

# Add a final cost equal to the total duration.
dircol.AddFinalCost(dircol.time())

initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
    [0., 4.], np.column_stack((initial_state, final_state)))  # yapf: disable
dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

result = Solve(dircol)
assert result.is_success()

fig, ax = plt.subplots(2, 1)


# Animate the results.
# TODO(russt): Add some helper methods to make this workflow cleaner.
def animate(plant, x_trajectory):
    builder = DiagramBuilder()
    source = builder.AddSystem(TrajectorySource(x_trajectory))
    builder.AddSystem(scene_graph)
    pos_to_pose = builder.AddSystem(
        MultibodyPositionToGeometryPose(plant, input_multibody_state=True))
    builder.Connect(source.get_output_port(0), pos_to_pose.get_input_port())
    builder.Connect(pos_to_pose.get_output_port(),
                    scene_graph.get_source_pose_port(plant.get_source_id()))

    visualizer = builder.AddSystem(
        PlanarSceneGraphVisualizer(scene_graph,
                                   xlim=[-2, 2],
                                   ylim=[-1.25, 2],
                                   ax=ax[0]))
    builder.Connect(scene_graph.get_pose_bundle_output_port(),
                    visualizer.get_input_port(0))

    simulator = Simulator(builder.Build())
    simulator.AdvanceTo(x_trajectory.end_time())


u_trajectory = dircol.ReconstructInputTrajectory(result)
times = np.linspace(u_trajectory.start_time(), u_trajectory.end_time(), 100)
u_lookup = np.vectorize(u_trajectory.value)
u_values = u_lookup(times)

ax[1].plot(times, u_values)
ax[1].set_xlabel("time (seconds)")
ax[1].set_ylabel("force (Newtons)")

x_trajectory = dircol.ReconstructStateTrajectory(result)
animate(plant, x_trajectory)

plt.show()
