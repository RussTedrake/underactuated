import array
import numpy as np
import matplotlib.pyplot as plt
from pydrake.all import (
    AddRandomInputs,
    BasicVector,
    DiagramBuilder,
    LeafSystem,
    PortDataType,
    RandomDistribution,
    Simulator
)
from underactuated.pyplot_visualizer import PyPlotVisualizer


def dynamics(x, w):
    return x - x**3 + .3*w


class SimpleStochasticSystem(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self._DeclareInputPort("noise",
                               PortDataType.kVectorValued,
                               1,
                               RandomDistribution.kGaussian)
        self._DeclareContinuousState(1)
        self._DeclareVectorOutputPort(BasicVector(1), self.CopyStateOut)

    # xdot(t) = x(t) - x^3(t) + w(t)
    def _DoCalcTimeDerivatives(self, context, derivatives):
        x = context.get_continuous_state_vector().GetAtIndex(0)
        w = self.EvalVectorInput(context, 0).GetAtIndex(0)
        xdot = dynamics(x, w)
        derivatives.get_mutable_vector().SetAtIndex(0, xdot)

    # y(t) = x(t)
    def CopyStateOut(self, context, output):
        x = context.get_continuous_state_vector().get_value()
        y = output.get_mutable_value()
        y[:] = x


# Note: This is a candidate for moving to a more central location.
class HistogramVisualizer(PyPlotVisualizer):
    def __init__(self, num_samples, bins, xlim, ylim, draw_timestep):
        PyPlotVisualizer.__init__(self, draw_timestep)
        for i in range(0, num_samples):
            self._DeclareInputPort(PortDataType.kVectorValued, 1)
        self.num_samples = num_samples
        self.bins = bins
        self.data = array.array('d', (0,)*num_samples)
        self.limits = xlim
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        self.patches = None

    def draw(self, context):
        if (self.patches):
            t = [p.remove() for p in self.patches]
        for i in range(0, self.num_samples):
            self.data[i] = self.EvalVectorInput(context, i).GetAtIndex(0)
        # TODO(russt): switch 'normed' to 'density' once the ubuntu version
        # supports it.
        count, bins, self.patches = self.ax.hist(self.data,
                                                 bins=self.bins,
                                                 range=self.limits,
                                                 normed=True,
                                                 facecolor='b')
        self.ax.set_title('t = ' + str(context.get_time()))


builder = DiagramBuilder()

num_particles = 1000
num_bins = 100
xlim = [-1.5, 1.5]
ylim = [-2, 4.5]
draw_timestep = .25
visualizer = builder.AddSystem(HistogramVisualizer(num_particles, num_bins,
                                                   xlim, ylim, draw_timestep))
x = np.linspace(xlim[0], xlim[1], 100)
visualizer.ax.plot(x, dynamics(x, 0), 'k', linewidth=2)

for i in range(0, num_particles):
    sys = builder.AddSystem(SimpleStochasticSystem())
    builder.Connect(sys.get_output_port(0), visualizer.get_input_port(i))

AddRandomInputs(.1, builder)

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_publish_every_time_step(False)
simulator.get_mutable_integrator().set_fixed_step_mode(True)
simulator.get_mutable_integrator().set_maximum_step_size(0.1)

simulator.StepTo(20)
plt.show()
