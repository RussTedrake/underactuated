
import numpy as np
import matplotlib.pyplot as plt

from pydrake.systems.analysis import Simulator

from plant import SLIPState, SpringLoadedInvertedPendulum

plant = SpringLoadedInvertedPendulum()

# Parameters from Geyer05, Figure 2.4
# Note: Geyer uses angle of attack = 90 - touchdown_angle
touchdown_angle = np.deg2rad(30)
Etilde = 1.61

s = SLIPState(np.zeros(8))
s.theta = touchdown_angle
s.r = 1

simulator = Simulator(plant)
context = simulator.get_mutable_context()
context.FixInputPort(0, [touchdown_angle])
context.SetAccuracy(1e-5)

zs = np.linspace(np.cos(touchdown_angle)+0.001, 0.95, 25)
zns = 0*zs
for i in range(len(zs)):
    s.z = zs[i]
    s.xdot = plant.apex_velocity_from_dimensionless_system_energy(Etilde, s.z)
    context.SetTime(0.)
    context.get_mutable_continuous_state_vector().SetFromVector(s[:])
    simulator.Initialize()
    # Note: With this duration, I sometimes get an extra "touchdown" after the
    # apex, which results in apex-touchdown; touchdown-takeoff-apex on the
    # console.  It's not a double reset, the consecutive touchdowns are two
    # different sims.
    simulator.AdvanceTo(0.6)
    zns[i] = plant.last_apex
    plant.last_apex = None

fix, ax = plt.subplots()
ax.plot(zs, zns)
ax.plot(zs, zs)
ax.axis('equal')
ax.set_xlim([zs[0], zs[-1]])
ax.set_ylim([zs[0], zs[-1]])
ax.set_xlabel('apex height z[n]')
ax.set_ylabel('apex height z[n+1]')

plt.show()
