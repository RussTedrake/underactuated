import numpy as np
import matplotlib.pyplot as plt

from pydrake.systems.pyplot_visualizer import PyPlotVisualizer
from pydrake.systems.framework import Context, PortDataType

_MIT_RED = "#A31F34"


class Brick(object):
    WIDTH = 3.
    HEIGHT = 1.

    def __init__(self):
        self.patch = plt.Rectangle((0.0, 0.0),
                                   self.WIDTH,
                                   self.HEIGHT,
                                   fc=_MIT_RED,
                                   ec="k")
        self.set_state(0.)

    def set_state(self, x):
        self.patch.set_x(x - self.WIDTH / 2)  # Center at x

    @classmethod
    def add_to_axes(cls, ax):
        brick = cls()
        ax.add_patch(brick.patch)
        return brick


class DoubleIntegratorVisualizer(PyPlotVisualizer):

    # Limits of view port
    XLIM = (-15., 15.)
    YLIM = (-6., 6.)
    TICK_DIMS = (0.2, 0.8)

    def __init__(self, ax=None, show=None):
        PyPlotVisualizer.__init__(self, ax=ax, show=show)
        self.DeclareInputPort("state", PortDataType.kVectorValued, 2)

        self.ax.set_xlim(*self.XLIM)
        self.ax.set_ylim(*self.YLIM)
        self.ax.set_aspect("auto")

        self._make_background()
        self.brick = Brick.add_to_axes(self.ax)

    def _make_background(self):
        # x-axis
        self.ax.plot(self.XLIM, np.zeros_like(self.XLIM), "k")
        # tick mark centered at the origin
        tick_pos = -0.5 * np.asarray(self.TICK_DIMS)
        self.ax.add_patch(plt.Rectangle(tick_pos, *self.TICK_DIMS, fc="k"))

    def draw(self, context):
        try:
            x = self.EvalVectorInput(context, 0).get_value()[0]
            self.ax.set_title("t = {:.1f}".format(context.get_time()))
        except TypeError:
            x = context[0]
        self.brick.set_state(x)
