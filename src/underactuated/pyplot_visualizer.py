# -*- coding: utf8 -*-

import numpy as np
import math
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from pydrake.all import (LeafSystem, PiecewisePolynomial, SignalLogger,
                         VectorSystem)

class PyPlotVisualizer(LeafSystem):
    '''
        Base class from planar visualization
        that relies on pyplot.

        In the configuration set up here,
        this visualizer provides one visualization
        window (self.fig) with axes (self.ax).

        Subclasses must:
        - During initialization, set up the figure
        bounds and register and input port
        with the appropriate message type.
        - Override the draw method to parse the
        input and draw the robot in the appropriate
        state.
    '''
    
    def __init__(self, draw_rate=60, facecolor=[1, 1, 1]):
        LeafSystem.__init__(self)

        self.set_name('pyplot_visualization')
        self.rate = draw_rate
        self._DeclarePeriodicPublish(1./draw_rate, 0.0)

        (self.fig, self.ax) = plt.subplots(facecolor=facecolor)
        self.ax.axis('equal')
        self.ax.axis('off')
        self.fig.show()

    def _DoPublish(self, context, event):
        self.draw(context)
        self.fig.canvas.draw()
        if plt.get_backend() == u'MacOSX':
            plt.pause(1e-10)

    def draw(self, context):
        print "SUBCLASSES MUST IMPLEMENT."

    def animate(self, log, resample=True, repeat=False):
        # log - a reference to a pydrake.systems.primitives.SignalLogger that
        # contains the plant state after running a simulation.
        # rate - the frequency of frames in the resulting animation
        # resample -- should we do a resampling operation to make
        # the samples more consistent in time? This can be disabled
        # if you know the sampling rate is exactly the rate you supply
        # as an argument.
        # repeat - should the resulting animation repeat?

        if type(log) is SignalLogger:
            t = log.sample_times()
            x = log.data()

            if resample:
                import scipy.interpolate

                t_resample = np.arange(0, t[-1], 1./self.rate)
                x = scipy.interpolate.interp1d(t, x, kind='linear', axis=1)(t_resample)  # noqa
                t = t_resample

        # TODO(russt): Replace PiecewisePolynomial with Trajectory if I ever
        # add the pydrake bindings for the base class.
        elif type(log) is PiecewisePolynomial:
            t = np.arange(log.start_time(), log.end_time(), 1./self.rate)
            x = np.hstack([log.value(time) for time in t])

        def animate_update(i):
            self.draw(x[:, i])

        ani = animation.FuncAnimation(self.fig,
                                      animate_update,
                                      t.shape[0],
                                      interval=1000./self.rate,
                                      repeat=repeat)
        return ani


class SliderSystem(VectorSystem):
    def __init__(self, ax, title, min, max):
        # 0 inputs, 1 output.
        VectorSystem.__init__(self, 0, 1)
        self.value = 0
        self.slider = Slider(ax, title, min, max, valinit=self.value)
        self.slider.on_changed(self.update)

    def update(self, val):
        self.value = val

    def _DoCalcVectorOutput(self, context, unused, unused2, output):
        output[:] = self.value
