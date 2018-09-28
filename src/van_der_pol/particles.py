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


class VanDerPolParticles(LeafSystem):
    def __init__(self, num_particles, mu=1.0):
        LeafSystem.__init__(self)
        self._DeclareInputPort("noise",
                               PortDataType.kVectorValued,
                               num_particles,
                               RandomDistribution.kGaussian)
        self._DeclareContinuousState(num_particles, num_particles, 0)
        self._DeclareVectorOutputPort(BasicVector(2*num_particles),
                                      self.CopyStateOut)
        self.num_particles = num_particles
        self.mu = mu

    # TODO(russt):  SetRandomState to  [-0.1144;2.0578] + 0.01*randn(...)

    def _DoCalcTimeDerivatives(self, context, derivatives):
        # TODO(russt):  Update this to get_position/velocity once those are
        # bound.
        x = context.get_continuous_state_vector().CopyToVector()
        q = x[:self.num_particles]
        qdot = x[self.num_particles:]
        w = self.EvalVectorInput(context, 0).CopyToVector()
        qddot = -self.mu * (q * q - 1) * qdot - q + .5 * w
        derivatives.get_mutable_vector().SetFromVector(np.concatenate((qdot,
                                                                       qddot)))

    # y(t) = x(t)
    def CopyStateOut(self, context, output):
        x = context.get_continuous_state_vector().get_value()
        y = output.get_mutable_value()
        y[:] = x


# Note: This is a candidate for moving to a more central location.
class Particle2DVisualizer(PyPlotVisualizer):
    def __init__(self, num_particles, xlim, ylim, draw_timestep):
        PyPlotVisualizer.__init__(self, draw_timestep)
        self._DeclareInputPort(PortDataType.kVectorValued, 2*num_particles)
        self.num_particles = num_particles
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        zero = array.array('d', (0,)*num_particles)
        self.lines, = self.ax.plot(zero, zero, 'b.')

    def draw(self, context):
        xy = self.EvalVectorInput(context, 0).CopyToVector()
        self.lines.set_xdata(xy[:self.num_particles])
        self.lines.set_ydata(xy[self.num_particles:])
        self.ax.set_title('t = ' + str(context.get_time()))


builder = DiagramBuilder()

num_particles = 5000
xlim = [-2.75, 2.75]
ylim = [-3.25, 3.25]
draw_timestep = .25
sys = builder.AddSystem(VanDerPolParticles(num_particles))
visualizer = builder.AddSystem(Particle2DVisualizer(num_particles, xlim,
                                                    ylim, draw_timestep))
builder.Connect(sys.get_output_port(0), visualizer.get_input_port(0))
AddRandomInputs(.1, builder)

# TODO(russt): Plot nominal limit cycle.

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_publish_every_time_step(False)
simulator.get_mutable_integrator().set_fixed_step_mode(True)
simulator.get_mutable_integrator().set_maximum_step_size(0.1)

simulator.StepTo(20)
plt.show()
