from IPython.display import display
from ipywidgets.widgets import FloatSlider

from pydrake.systems.framework import VectorSystem


class SliderSystem(VectorSystem):

    def __init__(self, min, max, value=0, description=""):
        # 0 inputs, 1 output.
        VectorSystem.__init__(self, 0, 1)
        self.slider = FloatSlider(value=value, description=description, min=min, max=max)
        display(self.slider)

    def DoCalcVectorOutput(self, context, unused, unused2, output):
        print(self.slider.value)
        output[:] = self.slider.value
