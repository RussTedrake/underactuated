import numpy as np
from pydrake.systems.framework import Context, PortDataType
from pydrake.systems.pyplot_visualizer import PyPlotVisualizer


class PendulumVisualizer(PyPlotVisualizer):
    a1 = 0.75
    ac1 = 0.75
    av = np.linspace(0, np.pi, 20)
    rb = 0.03
    hb = 0.07
    aw = 0.01
    base_x = rb * np.concatenate(([1], np.cos(av), [-1]))
    base_y = np.concatenate(([-hb], rb * np.sin(av), [-hb]))
    arm_x = np.concatenate((aw * np.sin(av - np.pi / 2), aw * np.sin(av + np.pi / 2)))
    arm_y = np.concatenate(
        (aw * np.cos(av - np.pi / 2), -a1 + aw * np.cos(av + np.pi / 2))
    )

    def __init__(self, ax=None, show=None):
        PyPlotVisualizer.__init__(self, ax=ax, show=show)
        self.set_name("pendulum_visualizer")
        self.DeclareInputPort("state", PortDataType.kVectorValued, 2)

        self.ax.set_xlim([-1.2, 1.2])
        self.ax.set_ylim([-1.2, 1.2])
        self.base = self.ax.fill(
            self.base_x,
            self.base_y,
            zorder=1,
            facecolor=(0.3, 0.6, 0.4),
            edgecolor="k",
        )

        # arm_x and arm_y are closed (last element == first element), but don't
        # pass the last element to the fill command, because it gets closed
        # anyhow (and we want the sizes to match for the update).
        self.arm = self.ax.fill(
            self.arm_x[0:-1],
            self.arm_y[0:-1],
            zorder=0,
            facecolor=(0.9, 0.1, 0),
            edgecolor="k",
        )
        self.center_of_mass = self.ax.plot(
            0, -self.ac1, zorder=1, color="b", marker="o", markersize=14
        )

    def draw(self, context):
        if isinstance(context, Context):
            theta = self.EvalVectorInput(context, 0).get_value()[0]
            self.ax.set_title("t = {:.1f}".format(context.get_time()))
        else:
            theta = context[0]
            self.ax.set_title("")

        path = self.arm[0].get_path()
        path.vertices[:, 0] = self.arm_x * np.cos(theta) - self.arm_y * np.sin(theta)
        path.vertices[:, 1] = self.arm_x * np.sin(theta) + self.arm_y * np.cos(theta)
        self.center_of_mass[0].set_data(
            self.ac1 * np.sin(theta), -self.ac1 * np.cos(theta)
        )
