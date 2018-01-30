# -*- coding: utf8 -*-

import scipy as sp
import scipy.spatial
import sys
import numpy as np
import random
import math
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import pydrake
from pydrake.parsers import PackageMap
from pydrake.rbtree import RigidBodyTree, Shape
import pydrake.rbtree

from pydrake.systems.framework import (
    DiagramBuilder,
    LeafSystem,
    PortDataType
    )
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.multibody.rigid_body_plant import RigidBodyPlant
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

        self._DeclareInputPort(PortDataType.kVectorValued, self.rbtree.get_num_positions() + self.rbtree.get_num_velocities())

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
        n_bodies = self.rbtree.get_num_bodies()-1
        color = iter(plt.cm.rainbow(np.linspace(0, 1, n_bodies)))
        for body_i in range(n_bodies):
            tf = self.rbtree.relativeTransform(kinsol, 0, body_i+1)
            viewPatches = self.getViewPatches(body_i, tf)
            c = next(color)
            for patch in viewPatches:
                self.body_fill_list += self.ax.fill(patch[0, :], patch[1, :], zorder=0, color=c, edgecolor='k', closed=False)

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
                    if 0 and geom.hasFaces():
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

    def draw(self, context):
        positions = self.EvalVectorInput(context, 0).get_value()[0:self.rbtree.get_num_positions()]

        kinsol = self.rbtree.doKinematics(positions)

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

    '''
    rbt = RigidBodyTree("Pendulum.urdf", floating_base_type=pydrake.rbtree.FloatingBaseType.kFixed)
    Tview = np.array([[1., 0., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]], dtype=np.float64)
    pbrv = PlanarRigidBodyVisualizer(rbt, Tview, [-1.2, 1.2], [-1.2, 1.2])
    '''

    rbt = RigidBodyTree("double_pendulum.urdf", floating_base_type=pydrake.rbtree.FloatingBaseType.kFixed)
    Tview = np.array([[1., 0., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]], dtype=np.float64)
    pbrv = PlanarRigidBodyVisualizer(rbt, Tview, [-3.0, 3.0], [-3.0, 3.0])
    

    '''
    # Doesn't work correctly, as it has no inputs hooked up.
    rbt = RigidBodyTree()
    world_frame = pydrake.rbtree.RigidBodyFrame("world_frame", rbt.world(), [0, 0, 0], [0, 0, 0])
    pydrake.rbtree.AddModelInstancesFromSdfString(open("double_pendulum.sdf", 'r').read(),
        pydrake.rbtree.FloatingBaseType.kFixed,
        world_frame,
        rbt)
    Tview = np.array([[0, 1., 0., 0.], [0., 0., 1., -3.], [0., 0., 0., 1.]], dtype=np.float64)
    pbrv = PlanarRigidBodyVisualizer(rbt, Tview, [-3., 3.], [-3., 3.])
    '''

    '''
    rbt = RigidBodyTree()
    world_frame = pydrake.rbtree.RigidBodyFrame("world_frame", rbt.world(), [0, 0, 0], [0, 0, 0])
    pmap = PackageMap()
    pmap.PopulateFromEnvironment("ROS_PACKAGE_PATH")
    pydrake.rbtree.AddModelInstanceFromUrdfStringSearchingInRosPackages(
        open("/home/gizatt/drake/multibody/rigid_body_plant/test/world.urdf", 'r').read(),
        pmap,
        "/home/gizatt/drake/examples/",
        pydrake.rbtree.FloatingBaseType.kRollPitchYaw,
        world_frame,
        rbt)
    val_start_frame = pydrake.rbtree.RigidBodyFrame("val_start_frame", rbt.world(), [0, 0, 2], [0, 0, 0])
    pydrake.rbtree.AddModelInstanceFromUrdfStringSearchingInRosPackages(
        open("/home/gizatt/drake/examples/valkyrie/urdf/urdf/valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf", 'r').read(),
        pmap,
        "/home/gizatt/drake/examples/",
        pydrake.rbtree.FloatingBaseType.kRollPitchYaw,
        val_start_frame,
        rbt)
    Tview = np.array([[1., 0., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]], dtype=np.float64)
    pbrv = PlanarRigidBodyVisualizer(rbt, Tview, [-2.0, 2.0], [-0.25, 3.0])
    '''

    rbplant = RigidBodyPlant(rbt)

    builder = DiagramBuilder()
    rbplant_sys = builder.AddSystem(rbplant)

    torque = 1.0
    if (len(sys.argv)>1):
        torque = float(sys.argv[1])
    torque_system = builder.AddSystem(ConstantVectorSource(np.ones((rbt.get_num_actuators(), 1))*torque))
    builder.Connect(torque_system.get_output_port(0),
                            rbplant_sys.get_input_port(0))
    print('Simulating with constant torque = ' + str(torque) + ' Newton-meters')

    visualizer = builder.AddSystem(pbrv)
    builder.Connect(rbplant_sys.get_output_port(0), visualizer.get_input_port(0))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # TODO(russt): Clean up state vector access below.
    state = simulator.get_mutable_context().get_mutable_state()\
                     .get_mutable_continuous_state().get_mutable_vector()

    print state.size()
    initial_state = np.zeros((rbt.get_num_positions() + rbt.get_num_velocities(), 1))
    initial_state[0] = 1.0
    state.SetFromVector(initial_state)

    simulator.StepTo(10.0)

    print(state.CopyToVector())