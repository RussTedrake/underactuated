import argparse
import matplotlib.pyplot as plt

from pydrake.all import (ConstantVectorSource,
                         DiagramBuilder,
                         SignalLogger,
                         Simulator)
from pydrake.examples.compass_gait import (CompassGait)

# TODO(russt): combine this with simulate.py if the set_publish_every_timestep
# semantics get cleaned up (drake #7845).

builder = DiagramBuilder()
compass_gait = builder.AddSystem(CompassGait())

parser = argparse.ArgumentParser()
parser.add_argument("-T", "--duration",
                    type=float,
                    help="Duration to run sim.",
                    default=10.0)
args = parser.parse_args()

hip_torque = builder.AddSystem(ConstantVectorSource([0.0]))
builder.Connect(hip_torque.get_output_port(0), compass_gait.get_input_port(0))

logger = builder.AddSystem(SignalLogger(14))
builder.Connect(compass_gait.get_output_port(1), logger.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_publish_every_time_step(True)
context = simulator.get_mutable_context()
context.SetAccuracy(1e-4)
context.SetContinuousState([0., 0., 0.4, -2.])

simulator.AdvanceTo(args.duration)

plt.figure()
plt.plot(logger.data()[4, :], logger.data()[11, :])
plt.xlabel('left leg angle')
plt.ylabel('left leg angular velocity')

plt.show()
