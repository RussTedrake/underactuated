import math
import numpy as np
import matplotlib.pyplot as plt

from pydrake.examples.pendulum import (PendulumPlant, PendulumState)
from pydrake.all import (DirectCollocation, PiecewisePolynomial,
                         SolutionResult)
from visualizer import PendulumVisualizer

plant = PendulumPlant()
context = plant.CreateDefaultContext()

dircol = DirectCollocation(plant, context, num_time_samples=21,
                           minimum_timestep=0.2, maximum_timestep=0.5)

dircol.AddEqualTimeIntervalsConstraints()

torque_limit = 3.0  # N*m.
u = dircol.input()
dircol.AddConstraintToAllKnotPoints(-torque_limit <= u[0])
dircol.AddConstraintToAllKnotPoints(u[0] <= torque_limit)

initial_state = PendulumState()
initial_state.set_theta(0.0)
initial_state.set_thetadot(0.0)
dircol.AddBoundingBoxConstraint(initial_state.get_value(),
                                initial_state.get_value(),
                                dircol.initial_state())
# More elegant version is blocked on drake #8315:
# dircol.AddLinearConstraint(dircol.initial_state()
#                               == initial_state.get_value())

final_state = PendulumState()
final_state.set_theta(math.pi)
final_state.set_thetadot(0.0)
dircol.AddBoundingBoxConstraint(final_state.get_value(),
                                final_state.get_value(),
                                dircol.final_state())
# dircol.AddLinearConstraint(dircol.final_state() == final_state.get_value())

R = 10  # Cost on input "effort".
dircol.AddRunningCost(R*u[0]**2)

initial_x_trajectory = \
    PiecewisePolynomial.FirstOrderHold([0., 4.],
                                       [initial_state.get_value(),
                                        final_state.get_value()])
dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

result = dircol.Solve()
assert(result == SolutionResult.kSolutionFound)

x_trajectory = dircol.ReconstructStateTrajectory()

vis = PendulumVisualizer()
ani = vis.animate(x_trajectory, repeat=True)

x_knots = np.hstack([x_trajectory.value(t) for t in
                     np.linspace(x_trajectory.start_time(),
                                 x_trajectory.end_time(), 100)])
plt.figure()
plt.plot(x_knots[0, :], x_knots[1, :])

plt.show()
