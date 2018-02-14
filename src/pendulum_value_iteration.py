
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

from pydrake.all import (DiagramBuilder, Simulator, VectorSystem)
from pydrake.examples.pendulum import PendulumPlant
from pydrake.systems.controllers import (
    DynamicProgrammingOptions, FittedValueIteration)
from underactuated import PendulumVisualizer


plant = PendulumPlant()
simulator = Simulator(plant)
options = DynamicProgrammingOptions()

def min_time_cost(context):
    x = context.get_continuous_state_vector().CopyToVector()
    x[0] = x[0] - math.pi
    if x.dot(x) < .05:
        return 0.;
    return 1.;


def quadratic_regulator_cost(context):
    x = context.get_continuous_state_vector().CopyToVector()
    x[0] = x[0] - math.pi
    u = plant.EvalVectorInput(context, 0).CopyToVector()
    return 2*x.dot(x) + u.dot(u)


if (False):
    cost_function = min_time_cost
    input_limit = 1.
    options.convergence_tol = 0.001
else:
    cost_function = quadratic_regulator_cost
    input_limit = 3.
    options.convergence_tol = 0.1

qbins = np.linspace(0., 2. * math.pi, 51)
qdotbins = np.linspace(-10., 10., 51)
state_grid = [set(qbins), set(qdotbins)]
options.state_indices_with_periodic_boundary_conditions = {0}

input_mesh = [set(np.linspace(-input_limit, input_limit, 9))]
timestep = 0.01

[Q, Qdot] = np.meshgrid(qbins, qdotbins)
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_xlabel("theta")
ax.set_ylabel("thetadot")

fig2 = plt.figure()
ax2 = fig2.gca(projection='3d')
ax2.set_xlabel("q")
ax2.set_ylabel("qdot")

def draw(iteration, mesh, cost_to_go, policy):
    # Drawing is slow, don't draw every frame.
    if iteration % 10 != 0:
        return
    plt.title("iteration " + str(iteration))
    J = np.reshape(cost_to_go, Q.shape)
    surf = ax.plot_surface(Q, Qdot, J, rstride=1, cstride=1,
                           cmap=cm.jet)

    Pi = np.reshape(policy, Q.shape)
    surf2 = ax2.plot_surface(Q, Qdot, Pi, rstride=1, cstride=1, cmap=cm.jet)

    plt.draw_all()
    plt.pause(0.00001)
    surf.remove()
    surf2.remove()

options.visualization_callback = draw

policy, cost_to_go = FittedValueIteration(simulator,
                                          cost_function,
                                          state_grid, input_mesh, timestep, options)

J = np.reshape(cost_to_go, Q.shape)
surf = ax.plot_surface(Q, Qdot, J, rstride=1, cstride=1,
                       cmap=cm.jet)

# Animate the resulting policy.
builder = DiagramBuilder()
pendulum = builder.AddSystem(PendulumPlant())

#TODO(russt): add wrap-around logic to barycentric mesh (so the policy has it, too)
class WrapTheta(VectorSystem):
    def __init__(self):
        VectorSystem.__init__(self, 2, 2)

    def _DoCalcVectorOutput(self, context, input, state, output):
        output[:] = input
        twoPI = 2.*math.pi;
        output[0] = output[0] - twoPI * math.floor( output[0] / twoPI )


wrap = builder.AddSystem(WrapTheta())
builder.Connect(pendulum.get_output_port(0), wrap.get_input_port(0))
vi_policy = builder.AddSystem(policy)
builder.Connect(wrap.get_output_port(0), vi_policy.get_input_port(0))
builder.Connect(vi_policy.get_output_port(0), pendulum.get_input_port(0))

visualizer = builder.AddSystem(PendulumVisualizer())
builder.Connect(pendulum.get_output_port(0), visualizer.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.Initialize()
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

state = simulator.get_mutable_context().get_continuous_state_vector()

initial_state = np.array([0.1, 0.0])
state.SetFromVector(initial_state)

simulator.StepTo(10.)

plt.show()

