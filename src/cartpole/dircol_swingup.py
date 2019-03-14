import math
import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (DirectCollocation, FloatingBaseType,
                         PiecewisePolynomial, RigidBodyTree, RigidBodyPlant,
                         Solve)
from underactuated import (FindResource, PlanarRigidBodyVisualizer)

tree = RigidBodyTree(FindResource("cartpole/cartpole.urdf"),
                     FloatingBaseType.kFixed)
plant = RigidBodyPlant(tree)
context = plant.CreateDefaultContext()

dircol = DirectCollocation(plant, context, num_time_samples=21,
                           minimum_timestep=0.1, maximum_timestep=0.4)

dircol.AddEqualTimeIntervalsConstraints()

initial_state = (0., 0., 0., 0.)
dircol.AddBoundingBoxConstraint(initial_state, initial_state,
                                dircol.initial_state())
# More elegant version is blocked on drake #8315:
# dircol.AddLinearConstraint(dircol.initial_state() == initial_state)

final_state = (0., math.pi, 0., 0.)
dircol.AddBoundingBoxConstraint(final_state, final_state,
                                dircol.final_state())
# dircol.AddLinearConstraint(dircol.final_state() == final_state)

R = 10  # Cost on input "effort".
u = dircol.input()
dircol.AddRunningCost(R*u[0]**2)

# Add a final cost equal to the total duration.
dircol.AddFinalCost(dircol.time())

initial_x_trajectory = \
    PiecewisePolynomial.FirstOrderHold([0., 4.],
                                       np.column_stack((initial_state,
                                                        final_state)))
dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

result = Solve(dircol)
assert result.is_success()

x_trajectory = dircol.ReconstructStateTrajectory(result)

fig, ax = plt.subplots(2, 1)

vis = PlanarRigidBodyVisualizer(tree, xlim=[-1.5, 1.5], ylim=[-.5, 2],
                                ax=ax[0])
ani = vis.animate(x_trajectory, repeat=True)

u_trajectory = dircol.ReconstructInputTrajectory(result)
times = np.linspace(u_trajectory.start_time(), u_trajectory.end_time(), 100)
u_lookup = np.vectorize(u_trajectory.value)
u_values = u_lookup(times)

ax[1].plot(times, u_values)
ax[1].set_xlabel('time (seconds)')
ax[1].set_ylabel('force (Newtons)')

plt.show()
