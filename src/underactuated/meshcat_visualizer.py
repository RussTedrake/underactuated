# -*- coding: utf8 -*-

import warnings

# Note: ImportWarning is silent by default in python 2.7, so just use the
# default warning channel instead.
warnings.warn("MeshcatVisualizer has moved to drake.  Please import from "
              "pydrake.systems.meshcat_visualizer instead.")
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
