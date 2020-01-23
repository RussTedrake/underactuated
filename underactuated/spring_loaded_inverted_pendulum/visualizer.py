import copy
import numpy as np

from pydrake.systems.framework import PortDataType
from pydrake.systems.pyplot_visualizer import PyPlotVisualizer

from plant import SLIPState


class SLIPVisualizer(PyPlotVisualizer):

    def __init__(self, ax=None):
        PyPlotVisualizer.__init__(self, ax=ax)
        self.DeclareInputPort(PortDataType.kVectorValued, 8)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(0, 2)
        self.ax.set_ylim(-2, 2)

        # Draw the ground.
        self.ax.plot([-50, 50], [0, 0], "k")

        a = np.linspace(0, 2 * np.pi, 50)
        radius = 0.1
        self.hip_fill = self.ax.fill(radius * np.sin(a),
                                     radius * np.cos(a),
                                     zorder=1,
                                     edgecolor="k",
                                     facecolor=[.6, .6, .6])
        self.hip = copy.copy(self.hip_fill[0].get_path().vertices)

        self.leg_line = [self.ax.plot([0, 0], [0, -1], "k")[0]]
        self.leg_data = [self.leg_line[0].get_xydata().T]
        for i in range(1, 13):
            self.leg_line.append(
                self.ax.plot(
                    0.1 * np.array(
                        [np.sin((i - 1) * np.pi / 2.),
                         np.sin(i * np.pi / 2.)]),
                    -.2 - .7 / 13 * np.array([i - 1, i]), "k")[0])
            self.leg_data.append(self.leg_line[i].get_xydata().T)

    def draw(self, context):
        state = SLIPState(self.EvalVectorInput(context, 0).CopyToVector())

        self.hip_fill[0].get_path().vertices[:, 0] = state.x + self.hip[:, 0]
        self.hip_fill[0].get_path().vertices[:, 1] = state.z + self.hip[:, 1]

        R = np.array([[np.cos(state.theta), -np.sin(state.theta)],
                      [np.sin(state.theta),
                       np.cos(state.theta)]])
        for i in range(len(self.leg_line)):
            self.leg_line[i].set_xdata(state.x +
                                       state.r * R[0, :].dot(self.leg_data[i]))
            self.leg_line[i].set_ydata(state.z +
                                       state.r * R[1, :].dot(self.leg_data[i]))

        self.ax.set_title("t = " + str(context.get_time()))
