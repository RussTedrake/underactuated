# -*- coding: utf8 -*-

import numpy as np
import math
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import pydrake
from pydrake.systems.framework import (
    DiagramBuilder,
    LeafSystem,
    PortDataType,
    VectorSystem,
    )

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
    
    def __init__(self, draw_rate=0.0333, facecolor=[1, 1, 1]):
        LeafSystem.__init__(self)

        self.set_name('pyplot_visualization')
        self._DeclarePeriodicPublish(draw_rate, 0.0)

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
