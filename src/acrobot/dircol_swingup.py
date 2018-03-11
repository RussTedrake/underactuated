import math
import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (DirectCollocation, FloatingBaseType,
                         PiecewisePolynomial, RigidBodyTree, RigidBodyPlant,
                         SolutionResult)
from pydrake.examples.acrobot import AcrobotPlant
from underactuated import (FindResource, PlanarRigidBodyVisualizer)

plant = AcrobotPlant()
context = plant.CreateDefaultContext()

kNumTimeSamples = 21
kMinimumTimeStep = 0.2
kMaximumTimeStep = 0.5

dircol = DirectCollocation(plant, context, kNumTimeSamples, kMinimumTimeStep,
                           kMaximumTimeStep)

dircol.AddEqualTimeIntervalsConstraints()

# Add input limits.
kTorqueLimit = 8.0  # N*m.
u = dircol.input()
dircol.AddConstraintToAllKnotPoints(-kTorqueLimit <= u[0])
dircol.AddConstraintToAllKnotPoints(u[0] <= kTorqueLimit)

initial_state = (0., 0., 0., 0.)
dircol.AddBoundingBoxConstraint(initial_state, initial_state,
                                dircol.initial_state())
# More elegant version is blocked on drake #8315:
# dircol.AddLinearConstraint(dircol.initial_state() == initial_state)

final_state = (math.pi, 0., 0., 0.)
dircol.AddBoundingBoxConstraint(final_state, final_state,
                                dircol.final_state())
# dircol.AddLinearConstraint(dircol.final_state() == final_state)

R = 10  # Cost on input "effort".
dircol.AddRunningCost(R*u[0]**2)

# Add a final cost equal to the total duration.
dircol.AddFinalCost(dircol.time())

initial_x_trajectory = \
    PiecewisePolynomial.FirstOrderHold([0., 4.], [initial_state, final_state])
dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

result = dircol.Solve()
assert(result == SolutionResult.kSolutionFound)

x_trajectory = dircol.ReconstructStateTrajectory()

tree = RigidBodyTree(FindResource("acrobot/acrobot.urdf"),
                     FloatingBaseType.kFixed)
vis = PlanarRigidBodyVisualizer(tree, xlim=[-4., 4.], ylim=[-4., 4.])
ani = vis.animate(x_trajectory, repeat=True)

u_trajectory = dircol.ReconstructInputTrajectory()
times = np.linspace(u_trajectory.start_time(), u_trajectory.end_time(), 100)
u_lookup = np.vectorize(u_trajectory.value)
u_values = u_lookup(times)

plt.figure()
plt.plot(times, u_values)
plt.xlabel('time (seconds)')
plt.ylabel('force (Newtons)')

plt.show()
