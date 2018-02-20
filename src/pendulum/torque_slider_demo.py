import argparse
from matplotlib.widgets import Slider
import numpy as np

from pydrake.all import (DiagramBuilder, Simulator, VectorSystem)
from pydrake.examples.pendulum import PendulumPlant
from underactuated import PendulumVisualizer


class TorqueSlider(VectorSystem):
    def __init__(self, ax):
        # 0 inputs, 1 output.
        VectorSystem.__init__(self, 0, 1)
        self.value = 0
        self.slider = Slider(ax, 'Torque', -5, 5, valinit=self.value)
        self.slider.on_changed(self.update)

    def update(self, val):
        self.value = val

    def _DoCalcVectorOutput(self, context, unused, unused2, torque):
        torque[:] = self.value


builder = DiagramBuilder()
pendulum = builder.AddSystem(PendulumPlant())

parser = argparse.ArgumentParser()
parser.add_argument("-T", "--duration",
                    type=float,
                    help="Duration to run sim.",
                    default=10000.0)
args = parser.parse_args()

visualizer = builder.AddSystem(PendulumVisualizer())
builder.Connect(pendulum.get_output_port(0), visualizer.get_input_port(0))

ax = visualizer.fig.add_axes([.2, .95, .6, .025])
torque_system = builder.AddSystem(TorqueSlider(ax))
builder.Connect(torque_system.get_output_port(0),
                pendulum.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

state = simulator.get_mutable_context().get_mutable_continuous_state_vector()
state.SetFromVector([1., 0.])

simulator.StepTo(args.duration)
