# -*- coding: utf8 -*-

import scipy as sp
import scipy.spatial
import numpy as np
import math
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import pydrake
from pydrake.rbtree import RigidBodyTree, Shape, AddModelInstancesFromSdfString
import pydrake.rbtree

from pydrake.systems.framework import (
    DiagramBuilder,
    LeafSystem,
    PortDataType
    )
import pydrake.multibody
import time

from pyplot_visualizer import PyPlotVisualizer
class PlanarRigidBodyVisualizer(PyPlotVisualizer):
    '''
        Given a RigidBodyPlant and a view plane,
        provides a view of the robot by projecting
        all geometry onto the view plane.

        This is intended to be
        used for robots that operate in the
        plane, and won't render out-of-plane
        transformations correctly. (It precomputes
        how geometry looks on the projected plane
        and assumes it won't be rotated out of
        plane.)

        view_origin, x_dir, and y_dir define
        a 2D coordinate system in which
        all points will be projected.

    '''

    def __init__(self, rbtree, Tview, xlim = [-10., 10], ylim = [-10, 10]):
        PyPlotVisualizer.__init__(self)
        self.set_name('planar_rigid_body_visualizer')

        self.rbtree = rbtree
        self.Tview = Tview

        print "Spawning for tree with %d actuators" % (self.rbtree.get_num_actuators())

        self._DeclareInputPort(PortDataType.kVectorValued, self.rbtree.get_num_actuators())

        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)

        # Populate body patches
        self.buildViewPatches()

        # Populate the body fill list -- which requires doing most of
        # a draw pass, but with an ax.fill() command rather than
        # an in-place replacement of vertex positions.
        self.body_fill_list = []
        q0 = np.zeros((self.rbtree.get_num_positions(),))
        kinsol = self.rbtree.doKinematics(q0)

        for body_i in range(self.rbtree.get_num_bodies()-1):
            tf = self.rbtree.relativeTransform(kinsol, 0, body_i+1)
            viewPatches = self.getViewPatches(body_i, tf)
            for patch in viewPatches:
                self.body_fill_list += self.ax.fill(patch[0, :], patch[1, :], zorder=0, color=(0.9, 0.1, 0), edgecolor='k', closed=False)

        self.draw(q0)

    def buildViewPatches(self):
        self.viewPatches = []
        for body_i in range(self.rbtree.get_num_bodies()-1):
            body = self.rbtree.get_body(body_i+1)
            visual_elements = body.get_visual_elements()
            this_body_patches = []
            for element in visual_elements:
                element_local_tf = element.getLocalTransform()
                if element.hasGeometry():
                    geom = element.getGeometry()

                    # Prefer to use faces when possible
                    if geom.hasFaces():
                        points = geom.getPoints()
                        tris = geom.getFaces()
                        for tri in tris:
                            patch = np.transpose(np.vstack([points[:, x] for x in tri]))
                            patch = np.vstack((patch, np.ones((1, patch.shape[1]))))
                            patch = np.dot(element_local_tf, patch)
                            # Project into 2D
                            patch = np.dot(self.Tview, patch)
                            # No convhull necessary, already appropriately ordered
                            this_body_patches.append(patch)
                    else:
                        # Placeholder until polymorphic Geometry wrapping works
                        patch = geom.getPoints()
                        patch = np.vstack((patch, np.ones((1, patch.shape[1]))))
                        patch = np.dot(element_local_tf, patch)
                        # Project into 2D
                        patch = np.dot(self.Tview, patch)

                        # Take convhull
                        if patch.shape[1] > 3:
                            hull = sp.spatial.ConvexHull(np.transpose(patch[0:2, :]))
                            patch = np.transpose(np.vstack([patch[:, v] for v in hull.vertices]))
                        this_body_patches.append(patch)

            self.viewPatches.append(this_body_patches)


    # Pulls out the view patch verts for the given body index
    def getViewPatches(self, body_i, tf):
        projected_tf = np.dot(np.dot(self.Tview, tf), np.linalg.pinv(self.Tview))
        return [np.dot(projected_tf, patch)[0:2] for patch in self.viewPatches[body_i]]

    def draw(self, state_vec):
        kinsol = self.rbtree.doKinematics(state_vec)

        body_fill_index = 0
        for body_i in range(self.rbtree.get_num_bodies()-1):
            tf = self.rbtree.relativeTransform(kinsol, 0, body_i+1)
            viewPatches = self.getViewPatches(body_i, tf)
            for patch in viewPatches:
                self.body_fill_list[body_fill_index].get_path().vertices[:, :] = np.transpose(patch)
                body_fill_index += 1

if __name__ == "__main__":
    np.set_printoptions(precision=5, suppress=True)

    # TView elements:
    # [ <x axis select> x_axis_shift
    #   <y axis select> y_axis_shift
    #   0, 0, 0 1]  homogenizer

    #rbt = RigidBodyTree("Pendulum.urdf", floating_base_type=pydrake.rbtree.FloatingBaseType.kFixed)
    #Tview = np.array([[1., 0., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]], dtype=np.float64)
    #pbrv = PlanarRigidBodyVisualizer(rbt, Tview, [-1.2, 1.2], [-1.2, 1.2])
    
    rbt = RigidBodyTree("double_pendulum.urdf", floating_base_type=pydrake.rbtree.FloatingBaseType.kFixed)
    Tview = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 0., 1.]], dtype=np.float64)
    pbrv = PlanarRigidBodyVisualizer(rbt, Tview, [-1.2, 1.2], [-1.2, 1.2])
    

    #rbt = RigidBodyTree()
    #world_frame = pydrake.rbtree.RigidBodyFrame("world_frame", rbt.world(), [0, 0, 0], [0, 0, 0])
    #AddModelInstancesFromSdfString(open("double_pendulum.sdf", 'r').read(),
    #    pydrake.rbtree.FloatingBaseType.kFixed,
    #    world_frame,
    #    rbt)
    #Tview = np.array([[0, 1., 0., 0.], [0., 0., 1., -3.], [0., 0., 0., 1.]], dtype=np.float64)
    #pbrv = PlanarRigidBodyVisualizer(rbt, Tview, [-3., 3.], [-3., 3.])

    for i in range(1000):
        q = np.array([math.pi*math.sin(i*0.01*(k+1)) for k in range(rbt.get_num_positions())])
        pbrv._DoPublish(q, None)