import numpy as np

from pydrake.systems.framework import LeafSystem_, PortDataType
from pydrake.systems.pyplot_visualizer import PyPlotVisualizer
from pydrake.systems.scalar_conversion import TemplateSystem

# Note: In order to use the Python system with drake's autodiff features, we
# have to add a little "TemplateSystem" boilerplate (for now).  For details,
# see https://drake.mit.edu/pydrake/pydrake.systems.scalar_conversion.html


# TODO(russt): Clean this up pending any resolutions on
#  https://github.com/RobotLocomotion/drake/issues/10745
@TemplateSystem.define("Quadrotor2D_")
def Quadrotor2D_(T):

    class Impl(LeafSystem_[T]):

        def _construct(self, converter=None):
            LeafSystem_[T].__init__(self, converter)
            # two inputs (thrust)
            self.DeclareVectorInputPort("u", 2)
            # three positions, three velocities
            state_index = self.DeclareContinuousState(3, 3, 0)
            # six outputs (full state)
            self.DeclareStateOutputPort("x", state_index)

            # parameters based on [Bouadi, Bouchoucha, Tadjine, 2007]
            self.length = 0.25  # length of rotor arm
            self.mass = 0.486  # mass of quadrotor
            self.inertia = 0.00383  # moment of inertia
            self.gravity = 9.81  # gravity

        def _construct_copy(self, other, converter=None):
            Impl._construct(self, converter=converter)

        def DoCalcTimeDerivatives(self, context, derivatives):
            x = context.get_continuous_state_vector().CopyToVector()
            u = self.EvalVectorInput(context, 0).CopyToVector()
            q = x[:3]
            qdot = x[3:]
            qddot = np.array([
                -np.sin(q[2]) / self.mass * (u[0] + u[1]),
                np.cos(x[2]) / self.mass * (u[0] + u[1]) - self.gravity,
                self.length / self.inertia * (u[0] - u[1])
            ])
            derivatives.get_mutable_vector().SetFromVector(
                np.concatenate((qdot, qddot)))

    return Impl


Quadrotor2D = Quadrotor2D_[None]  # Default instantiation


class Quadrotor2DVisualizer(PyPlotVisualizer):

    def __init__(self, ax=None, show=None):
        PyPlotVisualizer.__init__(self, ax=ax, show=show)
        self.DeclareInputPort("state", PortDataType.kVectorValued, 6)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-1, 1)

        self.length = .25  # moment arm (meters)

        self.base = np.vstack((1.2 * self.length * np.array([1, -1, -1, 1, 1]),
                               0.025 * np.array([1, 1, -1, -1, 1])))
        self.pin = np.vstack((0.005 * np.array([1, 1, -1, -1, 1]),
                              .1 * np.array([1, 0, 0, 1, 1])))
        a = np.linspace(0, 2 * np.pi, 50)
        self.prop = np.vstack(
            (self.length / 1.5 * np.cos(a), .1 + .02 * np.sin(2 * a)))

        # yapf: disable
        self.base_fill = self.ax.fill(
            self.base[0, :], self.base[1, :], zorder=1, edgecolor="k",
            facecolor=[.6, .6, .6])
        self.left_pin_fill = self.ax.fill(
            self.pin[0, :], self.pin[1, :], zorder=0, edgecolor="k",
            facecolor=[0, 0, 0])
        self.right_pin_fill = self.ax.fill(
            self.pin[0, :], self.pin[1, :], zorder=0, edgecolor="k",
            facecolor=[0, 0, 0])
        self.left_prop_fill = self.ax.fill(
            self.prop[0, :], self.prop[0, :], zorder=0, edgecolor="k",
            facecolor=[0, 0, 1])
        self.right_prop_fill = self.ax.fill(
            self.prop[0, :], self.prop[0, :], zorder=0, edgecolor="k",
            facecolor=[0, 0, 1])
        # yapf: enable

    def draw(self, context):
        x = self.EvalVectorInput(context, 0).CopyToVector()
        R = np.array([[np.cos(x[2]), -np.sin(x[2])],
                      [np.sin(x[2]), np.cos(x[2])]])

        p = np.dot(R, self.base)
        self.base_fill[0].get_path().vertices[:, 0] = x[0] + p[0, :]
        self.base_fill[0].get_path().vertices[:, 1] = x[1] + p[1, :]

        p = np.dot(R, np.vstack(
            (-self.length + self.pin[0, :], self.pin[1, :])))
        self.left_pin_fill[0].get_path().vertices[:, 0] = x[0] + p[0, :]
        self.left_pin_fill[0].get_path().vertices[:, 1] = x[1] + p[1, :]
        p = np.dot(R, np.vstack((self.length + self.pin[0, :], self.pin[1, :])))
        self.right_pin_fill[0].get_path().vertices[:, 0] = x[0] + p[0, :]
        self.right_pin_fill[0].get_path().vertices[:, 1] = x[1] + p[1, :]

        p = np.dot(R,
                   np.vstack((-self.length + self.prop[0, :], self.prop[1, :])))
        self.left_prop_fill[0].get_path().vertices[:, 0] = x[0] + p[0, :]
        self.left_prop_fill[0].get_path().vertices[:, 1] = x[1] + p[1, :]

        p = np.dot(R, np.vstack(
            (self.length + self.prop[0, :], self.prop[1, :])))
        self.right_prop_fill[0].get_path().vertices[:, 0] = x[0] + p[0, :]
        self.right_prop_fill[0].get_path().vertices[:, 1] = x[1] + p[1, :]

        self.ax.set_title("t = {:.1f}".format(context.get_time()))
