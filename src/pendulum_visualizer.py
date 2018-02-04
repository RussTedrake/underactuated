# -*- coding: utf8 -*-

import numpy as np
import math
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import pydrake
from pydrake.systems.framework import (
    Context,
    DiagramBuilder,
    LeafSystem,
    PortDataType,
    )
import scipy.interpolate

from pyplot_visualizer import PyPlotVisualizer
class PendulumVisualizer(PyPlotVisualizer):
    a1 = 0.75
    ac1 = 0.75
    av = np.linspace(0,math.pi,20)
    rb = .03
    hb=.07
    aw = .01
    base_x = rb*np.concatenate(([1], np.cos(av), [-1]))
    base_y = np.concatenate(([-hb], rb*np.sin(av), [-hb]))
    arm_x = np.concatenate((aw*np.sin(av-math.pi/2), aw*np.sin(av+math.pi/2)))
    arm_y = np.concatenate((aw*np.cos(av-math.pi/2), -a1+aw*np.cos(av+math.pi/2)))

    def __init__(self):
        PyPlotVisualizer.__init__(self)
        self.set_name('pendulum_visualizer')
        self._DeclareInputPort(PortDataType.kVectorValued, 2)

        self.ax.set_xlim([-1.2, 1.2])
        self.ax.set_ylim([-1.2, 1.2])
        self.base = self.ax.fill(self.base_x, self.base_y, zorder=1, color=(.3, .6, .4), edgecolor='k')
        # arm_x and arm_y are closed (last element == first element), but don't pass the
        # last element to the fill command, because it gets closed anyhow (and we want the
        # sizes to match for the update).
        self.arm = self.ax.fill(self.arm_x[0:-1], self.arm_y[0:-1], zorder=0, color=(.9, .1, 0), edgecolor='k')
        self.center_of_mass = self.ax.plot(0, -self.ac1, zorder=1, color='b', marker='o', markersize=14)

    def draw(self, context):
        if isinstance(context, Context):
            theta = self.EvalVectorInput(context, 0).get_value()[0]
            self.ax.set_title('t = {:.1f}'.format(context.get_time()))
        else:
            theta = context
            self.ax.set_title('')

        path = self.arm[0].get_path()
        path.vertices[:,0] = self.arm_x*math.cos(theta)-self.arm_y*math.sin(theta)
        path.vertices[:,1] = self.arm_x*math.sin(theta)+self.arm_y*math.cos(theta)
        self.center_of_mass[0].set_data(self.ac1*math.sin(theta),-self.ac1*math.cos(theta))

    def animate(self, log, rate, resample=True, repeat=False):
        # log - a reference to a pydrake.systems.primitives.SignalLogger that
        # constains the plant state after running a simulation.
        # rate - the frequency of frames in the resulting animation
        # resample -- should we do a resampling operation to make
        # the samples more consistent in time? This can be disabled
        # if you know the sampling rate is exactly the rate you supply
        # as an argument.
        # repeat - should the resulting animation repeat?
        t = log.sample_times()
        x = log.data()

        if resample:
            t_resample = np.arange(0, t[-1], 1./rate)
            x = scipy.interpolate.interp1d(t, x, kind='linear', axis=1)(t_resample)
            t = t_resample

        animate_update = lambda i: self.draw(x[0, i])
        ani = animation.FuncAnimation(self.fig, animate_update, t.shape[0], interval=1000./rate, repeat=repeat)
        return ani