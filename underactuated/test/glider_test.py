import numpy as np
import unittest
import matplotlib.pyplot as plt

from pydrake.all import (ConstantVectorSource, ConnectDrakeVisualizer,
                         DiagramBuilder, ConnectPlanarSceneGraphVisualizer,
                         SceneGraph)
from underactuated.glider import GliderGeometry, GliderState


class TestGlider(unittest.TestCase):

    def test_visualizer(self):
        builder = DiagramBuilder()
        scene_graph = builder.AddSystem(SceneGraph())
        state = GliderState(np.zeros((7,)))
        state.x = -.3
        state.z = 0.1
        state.pitch = 0.2
        state.elevator = .4
        source = builder.AddSystem(ConstantVectorSource(state[:]))
        geom = GliderGeometry.AddToBuilder(builder, source.get_output_port(0),
                                           scene_graph)
        plt_vis = ConnectPlanarSceneGraphVisualizer(builder, scene_graph)
        drake_viz = ConnectDrakeVisualizer(builder, scene_graph)
        diagram = builder.Build()

        context = diagram.CreateDefaultContext()
        diagram.Publish(context)


if __name__ == '__main__':
    unittest.main()
