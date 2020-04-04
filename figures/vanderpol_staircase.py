import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (Simulator, LeafSystem, WitnessFunctionDirection,
                         UnrestrictedUpdateEvent)


class VanDerPolOscillator(LeafSystem):

    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclareContinuousState(1, 1, 0)

        self.mu = 0.2
        self.last_poincare = None  # placeholder for writing return map result.

        self.poincare_witness = self.MakeWitnessFunction(
            "poincare", WitnessFunctionDirection.kNegativeThenNonNegative,
            self.poincare, UnrestrictedUpdateEvent(self.record_poincare))

    def poincare(self, context):
        return context.get_continuous_state_vector().GetAtIndex(0)

    def record_poincare(self, context, event, state):
        if self.last_poincare is None:
            self.last_poincare = context.get_continuous_state_vector(
            ).GetAtIndex(1)

    def DoGetWitnessFunctions(self, context):
        return [self.poincare_witness]

    def DoCalcTimeDerivatives(self, context, derivatives):
        q = context.get_continuous_state_vector().GetAtIndex(0)
        qdot = context.get_continuous_state_vector().GetAtIndex(1)

        qddot = -self.mu * (q * q - 1) * qdot - q

        derivatives.get_mutable_vector().SetFromVector([qdot, qddot])


plant = VanDerPolOscillator()
simulator = Simulator(plant)
context = simulator.get_mutable_context()
context.SetAccuracy(1e-4)


def retmap(qdot0):
    context.SetTime(0.)
    context.SetContinuousState([0, qdot0])
    simulator.Initialize()
    plant.last_poincare = None
    simulator.AdvanceTo(8.0)
    return plant.last_poincare if plant.last_poincare else 0.0


zs = np.linspace(0, 4, 25)
zns = 0 * zs
for i in range(len(zs)):
    zns[i] = retmap(zs[i])

fix, ax = plt.subplots()
ax.plot(zs, zns, 'b')
ax.plot(zs, zs, 'r')
ax.axis("equal")
ax.set_xlim([zs[0], zs[-1]])
ax.set_ylim([zs[0], zs[-1]])
ax.set_xlabel("qdot_p[n]")
ax.set_ylabel("qdot_p[n+1]")


def staircase(xp, n=3):
    for i in range(n):
        xpn = retmap(xp)
        plt.arrow(xp,
                  xp,
                  0,
                  xpn - xp,
                  head_width=0.05,
                  color='k',
                  head_starts_at_zero=False,
                  length_includes_head=True)
        plt.arrow(xp,
                  xpn,
                  xpn - xp,
                  0,
                  head_width=0.05,
                  color='k',
                  head_starts_at_zero=False,
                  length_includes_head=True)
        xp = xpn


staircase(.5, 4)
staircase(3.5, 2)

plt.savefig('vanderpol_staircase.svg')
plt.show()
