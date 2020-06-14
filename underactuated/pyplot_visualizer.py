from matplotlib.widgets import Slider
from matplotlib.animation import HTMLWriter

from pydrake.systems.framework import VectorSystem


class SliderSystem(VectorSystem):

    def __init__(self, ax, title, min, max):
        # 0 inputs, 1 output.
        VectorSystem.__init__(self, 0, 1)
        self.value = 0
        self.slider = Slider(ax, title, min, max, valinit=self.value)
        self.slider.on_changed(self.update)

    def update(self, val):
        self.value = val

    def DoCalcVectorOutput(self, context, unused, unused2, output):
        output[:] = self.value


def AdvanceToAndSaveAnimation(simulator, visualizer, time, filename):
    visualizer.start_recording()
    simulator.AdvanceTo(time)
    ani = visualizer.get_recording_as_animation()
    # Note: Wanted to use embed_frames=True, but it did not render
    # the image properly for me.
    writer = HTMLWriter()
    writer.frame_format = 'svg'
    ani.save(filename, writer=writer)
