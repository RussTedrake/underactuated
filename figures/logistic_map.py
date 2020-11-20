import matplotlib as mp
mp.use("Qt5Agg")  # apt install python3-pyqt5

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import StrMethodFormatter

from pydrake.all import (DiagramBuilder, LogOutput, Simulator,
                         SymbolicVectorSystem, Variable)

builder = DiagramBuilder()

x = Variable("x")
plant = builder.AddSystem(
    SymbolicVectorSystem(state=[x],
                         dynamics=[4 * x * (1 - x)],
                         output=[x],
                         time_period=1))
log = LogOutput(plant.get_output_port(0), builder)

diagram = builder.Build()
simulator = Simulator(diagram)
context = simulator.get_mutable_context()

random_state = np.random.RandomState()  # see drake #12632.
fig, ax = plt.subplots(2, 1)

for i in range(2):
    context.SetTime(0)
    context.SetDiscreteState([random_state.uniform()])
    log.reset()
    simulator.Initialize()
    simulator.AdvanceTo(200)

    ax[i].plot(log.sample_times(), log.data().T)
    ax[i].set_xlabel('n')
    ax[i].set_ylabel('x[n]')
    ax[i].set_xlim([0, 200])
    ax[i].set_ylim([0, 1])

plt.savefig('figures/logistic_map_rollouts.svg')

# Plot the actual map
x = np.linspace(0, 1, 100)
xn = 4 * x * (1 - x)
fig, ax = plt.subplots()
ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)
ax.plot(x, xn)
ax.axis('equal')
ax.plot([0, 1], [0, 1])
ax.set_xlim([0, 1])
ax.set_ylim([0, 1.01])
ax.set_xlabel('x[n]')
ax.set_ylabel('x[n+1]')

plt.savefig('figures/logistic_map.svg')

fig, ax = plt.subplots(1, 3, figsize=(10, 4))
x = np.linspace(0.01, 0.99, 100)
p0 = 1 + 0 * x
p1 = 1. / (2 * np.sqrt(1 - x))
p2 = np.sqrt(2) / (8 * np.sqrt(1 - x)) * (1. / np.sqrt(1 + np.sqrt(1 - x))
                                          + 1. / np.sqrt(1 - np.sqrt(1 - x)))
pstar = 1 / (np.pi * np.sqrt(x * (1 - x)))

ax[0].plot(x, p0)
ax[1].plot(x, p1)
ax[2].plot(x, p2)

for i in range(3):
    ax[i].plot(x, pstar, '--')
    ax[i].set_xlim([0, 1])
    ax[i].set_ylim([0, 2.2])
    ax[i].set_xlabel('x')
    ax[i].yaxis.set_major_formatter(
        StrMethodFormatter('{x:,.1f}'))  # No decimal places

ax[0].set_title('p₀(x)')
ax[1].set_title('p₁(x)')
ax[2].set_title('p₂(x)')
ax[0].set_ylabel('p(x)')
plt.tight_layout()

plt.savefig('figures/logistic_map_master.svg')

plt.show()
