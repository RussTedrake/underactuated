import numpy as np
import matplotlib.pyplot as plt

from pydrake.examples.rimless_wheel import RimlessWheel
from pydrake.all import DirectCollocation, Solve

plant = RimlessWheel()
context = plant.CreateDefaultContext()

params = context.get_numeric_parameter(0)
slope = params.slope()
alpha = np.pi / params.number_of_spokes()

dircol = DirectCollocation(plant,
                           context,
                           num_time_samples=15,
                           minimum_timestep=0.01,
                           maximum_timestep=0.1,
                           assume_non_continuous_states_are_fixed=True)

dircol.AddEqualTimeIntervalsConstraints()

dircol.AddConstraintToAllKnotPoints(dircol.state()[0] >= slope - alpha)
dircol.AddConstraintToAllKnotPoints(dircol.state()[0] <= slope + alpha)

dircol.AddConstraint(dircol.initial_state()[0] == slope - alpha)
dircol.AddConstraint(dircol.final_state()[0] == slope + alpha)

dircol.AddConstraint(dircol.initial_state()[1] == dircol.final_state()[1] *
                     np.cos(2 * alpha))

result = Solve(dircol)
assert result.is_success()

x_trajectory = dircol.ReconstructStateTrajectory(result)

x_knots = np.hstack([
    x_trajectory.value(t) for t in np.linspace(x_trajectory.start_time(),
                                               x_trajectory.end_time(), 100)
])

fig, ax = plt.subplots()
ax.plot(x_knots[0, :], x_knots[1, :])
ax.plot([x_knots[0, 0], x_knots[0, -1]], [x_knots[1, 0], x_knots[1, -1]], "--")

# Plot the energy contours.
nq = 151
nqd = 151
mgl = params.mass() * params.gravity() * params.length()
q = np.linspace(-0.5, 0.5, nq)
qd = np.linspace(-.5, 2, nqd)
Q, QD = np.meshgrid(q, qd)
Energy = .5 * params.mass() * params.length()**2 * QD**2 + mgl * np.cos(Q)
ax.contour(Q,
           QD,
           Energy,
           alpha=0.5,
           linestyles="dashed",
           colors="black",
           linewidths=0.5)

ax.set_xlabel("theta")
ax.set_ylabel("thetadot")
ax.axis([-0.5, 0.5, 0, 2])
ax.set_title("Limit Cycle of the Rimless Wheel (w/ contours of "
             "constant energy)")

plt.show()
