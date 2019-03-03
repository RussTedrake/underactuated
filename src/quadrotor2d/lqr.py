
import argparse
import numpy as np

from pydrake.systems.analysis import Simulator
from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.systems.framework import DiagramBuilder

from quadrotor2d import Quadrotor2D, Quadrotor2DVisualizer


parser = argparse.ArgumentParser()
parser.add_argument("-N", "--trials",
                    type=int,
                    help="Number of trials to run.",
                    default=5)
parser.add_argument("-T", "--duration",
                    type=float,
                    help="Duration to run each sim.",
                    default=4.0)
args = parser.parse_args()


def QuadrotorLQR(plant):
    context = plant.CreateDefaultContext()
    context.SetContinuousState(np.zeros([6, 1]))
    context.FixInputPort(0, plant.mass*plant.gravity/2.*np.array([1, 1]))

    Q = np.diag([10, 10, 10, 1, 1, (plant.length/2./np.pi)])
    R = np.array([[0.1, 0.05], [0.05, 0.1]])

    return LinearQuadraticRegulator(plant, context, Q, R)


builder = DiagramBuilder()

plant = builder.AddSystem(Quadrotor2D())
visualizer = builder.AddSystem(Quadrotor2DVisualizer())
builder.Connect(plant.get_output_port(0), visualizer.get_input_port(0))

controller = builder.AddSystem(QuadrotorLQR(plant))
builder.Connect(controller.get_output_port(0), plant.get_input_port(0))
builder.Connect(plant.get_output_port(0), controller.get_input_port(0))

diagram = builder.Build()

simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
context = simulator.get_mutable_context()


for i in range(args.trials):
    context.set_time(0.)
    context.SetContinuousState(np.random.randn(6,))
    simulator.Initialize()
    simulator.StepTo(args.duration)
