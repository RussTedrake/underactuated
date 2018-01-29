# -*- coding: utf8 -*-

import numpy as np
import math
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import pydrake
from pydrake.systems.framework import (
    DiagramBuilder,
    LeafSystem,
    PortDataType,
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
        - 

    '''
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

    def __init__(self, draw_rate=0.0333):
        LeafSystem.__init__(self)
        self.set_name('pyplot_visualization')
        self._DeclarePeriodicPublish(draw_rate, 0.0)

        (self.fig, self.ax) = plt.subplots()
        self.ax.axis('equal')
        self.ax.axis('off')
        self.fig.show()

    def _DoPublish(self, context, event):
        self.draw(context)
        # TODO(gizatt) Figure out how to remove this with appropriate
        # backend canvas draw or flip commands.
        plt.pause(0.0000001)

    def draw(self, context):
        print "SUBCLASSES MUST IMPLEMENT."
