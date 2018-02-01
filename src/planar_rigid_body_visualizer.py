# -*- coding: utf8 -*-

import argparse
import math
import random
import sys
import time
import os.path

import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import scipy as sp
import scipy.spatial
import scipy.interpolate

import pydrake
import pydrake.rbtree
from pydrake.rbtree import RigidBodyTree
from pydrake.parsers import PackageMap
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (
    DiagramBuilder,
    LeafSystem,
    PortDataType,
    Context
    )
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.multibody.rigid_body_plant import RigidBodyPlant
from pydrake.multibody.shapes import Shape

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
        plane. It'll *work* for out-of-plane
        transformations, but render things
        increasingly inaccurately.)

        Params:
        - Tview, xlim, and ylim set up view into scene.
        - facecolor is passed through to figure() and sets
        background color. Both color name strings and
        RGB triplets are allowed. Defaults to white.

        Specifics on view setup:

        TView specifies the view projection matrix,
        and should be a 3x4 matrix:
        [ <x axis select> x_axis_shift
          <y axis select> y_axis_shift
           0, 0, 0 1]  homogenizer

        e.g.

        [ 1 0 0 0.5
          0 1 0 0
          0 0 0 1]

        would give a top-down view (i.e squashing
        the z axis), and would shift things in the
        x axis positively by 0.5.

        xlim and ylim don't technically provide
        extra functionality, but I think it's easier
        to keep handle scaling with xlim and ylim
        and view plane selection and *maybe*
        offsetting with the projection matrix.
    '''

    def __init__(self, rbtree, Tview, xlim = [-1., 1], ylim = [-1, 1],
            facecolor=[1, 1, 1], use_random_colors=False):
        PyPlotVisualizer.__init__(self)
        self.set_name('planar_rigid_body_visualizer')

        self.rbtree = rbtree
        self.Tview = Tview
        self.Tview_pinv = np.linalg.pinv(self.Tview)

        print "Spawning PlanarRigidBodyVisualizer for tree with %d actuators" % (self.rbtree.get_num_actuators())

        self._DeclareInputPort(PortDataType.kVectorValued, self.rbtree.get_num_positions() + self.rbtree.get_num_velocities())

        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)

        # Populate body patches
        self.buildViewPatches(use_random_colors)

        # Populate the body fill list -- which requires doing most of
        # a draw pass, but with an ax.fill() command rather than
        # an in-place replacement of vertex positions to initialize
        # the draw patches.
        # The body fill list stores the ax patch objects in the
        # order they were spawned (i.e. by body, and then by
        # order of viewPatches). Drawing the tree should update them
        # by iterating over bodies and patches in the same order.
        self.body_fill_list = []
        q0 = np.zeros((self.rbtree.get_num_positions(),))
        kinsol = self.rbtree.doKinematics(q0)
        n_bodies = self.rbtree.get_num_bodies()-1
        for body_i in range(n_bodies):
            tf = self.rbtree.relativeTransform(kinsol, 0, body_i+1)
            viewPatches, viewColors = self.getViewPatches(body_i, tf)
            for patch, color in zip(viewPatches, viewColors):
                self.body_fill_list += self.ax.fill(patch[0, :], patch[1, :], 
                    zorder=0, color=color, edgecolor='k', closed=False)

    def buildViewPatches(self, use_random_colors):
        ''' Generates view patches. self.viewPatches stores a list
        of viewPatches for each body (starting at body id 1). A viewPatch
        is a list of 2D coordinates in counterclockwise order forming
        the boundary of a filled polygon representing a piece of visual
        geometry. '''
        self.viewPatches = []
        self.viewPatchColors = []

        # Spawn a random color generator, in case we need to pick
        # random colors for some bodies. Each body will be given
        # a unique color when using this random generator, with
        # each visual element of the body colored the same.
        n_bodies = self.rbtree.get_num_bodies()-1
        color = iter(plt.cm.rainbow(np.linspace(0, 1, n_bodies)))
        for body_i in range(n_bodies):
            body = self.rbtree.get_body(body_i+1)
            visual_elements = body.get_visual_elements()
            this_body_patches = []
            this_body_colors = []
            for element in visual_elements:
                element_local_tf = element.getLocalTransform()
                if element.hasGeometry():
                    geom = element.getGeometry()
                    # Prefer to use faces when possible.
                    if geom.hasFaces():
                        try:
                            points = geom.getPoints()
                            # Unnecessary if we're taking a convex hull
                            #tris = geom.getFaces()
                            #for tri in tris:
                            #    new_pts = np.transpose(np.vstack([points[:, x] for x in tri]))
                            #    patch.append(new_pts)
                            patch = points
                        except Exception as e:
                            print "Exception when loading tris from geometry: ", e
                    else:
                        geom_type = geom.getShape()
                        if geom_type == Shape.SPHERE:
                            # Sphere will return their center as their
                            # getPoint(). So generate a better patch by sampling
                            # points around the sphere surface in the view plane.
                            center = geom.getPoints()
                            sample_pts = np.arange(0., 2.*math.pi, 0.25)
                            patch = np.vstack([math.cos(pt)*self.Tview[0, 0:3] + math.sin(pt)*self.Tview[1, 0:3] for pt in sample_pts])
                            patch = np.transpose(patch)
                            patch *= geom.radius
                        else:
                            # Other geometry types will usually return their
                            # bounding box, which is good enough for basic
                            # visualization. (This could be improved for 
                            # capsules?)
                            patch = geom.getPoints()
                    # Convert to homogenous coords and move out of body frame
                    patch = np.vstack((patch, np.ones((1, patch.shape[1]))))
                    patch = np.dot(element_local_tf, patch)
                    # Project into 2D
                    patch = np.dot(self.Tview, patch)

                    # Take convhull of resulting points
                    if patch.shape[1] > 3:
                        hull = sp.spatial.ConvexHull(np.transpose(patch[0:2, :]))
                        patch = np.transpose(np.vstack([patch[:, v] for v in hull.vertices]))
                    this_body_patches.append(patch)
                    if use_random_colors:
                        this_body_colors.append(next(color))
                    else:
                        this_body_colors.append(element.getMaterial())

            self.viewPatches.append(this_body_patches)
            self.viewPatchColors.append(this_body_colors)


    def getViewPatches(self, body_i, tf):
        ''' Pulls out the view patch verts for the given body index after applying
            the appropriate TF '''
        projected_tf = np.dot(np.dot(self.Tview, tf), self.Tview_pinv)
        transformed_patches = [np.dot(projected_tf, patch)[0:2] for patch in self.viewPatches[body_i]]
        colors = self.viewPatchColors[body_i]
        return (transformed_patches, colors)

    def draw(self, context):
        ''' Evaluates the robot state and draws it.
            Can be passed either a raw state vector, or
            an input context.'''
        if isinstance(context, Context):
            positions = self.EvalVectorInput(context, 0).get_value()[0:self.rbtree.get_num_positions()]
        else:
            positions = context[0:self.rbtree.get_num_positions()]

        kinsol = self.rbtree.doKinematics(positions)

        body_fill_index = 0
        for body_i in range(self.rbtree.get_num_bodies()-1):
            tf = self.rbtree.relativeTransform(kinsol, 0, body_i+1)
            viewPatches, _ = self.getViewPatches(body_i, tf)
            for patch in viewPatches:
                self.body_fill_list[body_fill_index].get_path().vertices[:, :] = np.transpose(patch)
                body_fill_index += 1

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

        animate_update = lambda i: self.draw(x[:, i])
        ani = animation.FuncAnimation(self.fig, animate_update, t.shape[0], interval=1000./rate, repeat=repeat)
        return ani


def setupPendulumExample():
    rbt = RigidBodyTree("pendulum.urdf", floating_base_type=pydrake.rbtree.FloatingBaseType.kFixed)
    Tview = np.array([[1., 0., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]], dtype=np.float64)
    pbrv = PlanarRigidBodyVisualizer(rbt, Tview, [-1.2, 1.2], [-1.2, 1.2])
    return rbt, pbrv

def setupDoublePendulumExample():
    rbt = RigidBodyTree("double_pendulum.urdf", floating_base_type=pydrake.rbtree.FloatingBaseType.kFixed)
    Tview = np.array([[1., 0., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]], dtype=np.float64)
    pbrv = PlanarRigidBodyVisualizer(rbt, Tview, [-2.5, 2.5], [-2.5, 2.5], use_random_colors=True)
    return rbt, pbrv

def setupValkyrieExample():
    # Valkyrie Example
    rbt = RigidBodyTree()
    world_frame = pydrake.rbtree.RigidBodyFrame("world_frame", rbt.world(), [0, 0, 0], [0, 0, 0])
    pmap = PackageMap()
    pmap.PopulateFromEnvironment("ROS_PACKAGE_PATH")
    drake_base_path = os.path.expandvars("${DRAKE_RESOURCE_ROOT}")
    pydrake.rbtree.AddModelInstanceFromUrdfStringSearchingInRosPackages(
        open("plane.urdf", 'r').read(),
        pmap,
        drake_base_path + "/examples/",
        pydrake.rbtree.FloatingBaseType.kFixed,
        world_frame,
        rbt)
    val_start_frame = pydrake.rbtree.RigidBodyFrame("val_start_frame", rbt.world(), [0, 0, 1.5], [0, 0, 0])
    pydrake.rbtree.AddModelInstanceFromUrdfStringSearchingInRosPackages(
        open(drake_base_path + "/examples/valkyrie/urdf/urdf/valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf", 'r').read(),
        pmap,
        drake_base_path + "/examples/",
        pydrake.rbtree.FloatingBaseType.kRollPitchYaw,
        val_start_frame,
        rbt)
    Tview = np.array([[1., 0., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]], dtype=np.float64)
    pbrv = PlanarRigidBodyVisualizer(rbt, Tview, [-2.0, 2.0], [-0.25, 3.0], use_random_colors=True)
    return rbt, pbrv

if __name__ == "__main__":
    # Usage demo: load a URDF, rig it up with a constant torque input, and draw it.


    np.set_printoptions(precision=5, suppress=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--torque",
                        type=float,
                        help="Constant torque to apply to all joints.",
                        default=1.0)
    parser.add_argument("-T", "--time", 
                        type=float,
                        help="Duration to run sim.",
                        default=1.0)
    parser.add_argument("-m", "--models",
                        type=str,
                        nargs="*",
                        help="Models to run, at least one of [pend, dpend, val]",
                        default=["dpend"])
    args = parser.parse_args()

    for model in args.models:
        if model == "pend":
            rbt, pbrv = setupPendulumExample()
            timestep = 0.0
            set_initial_state = True
        elif model == "dpend":
            rbt, pbrv = setupDoublePendulumExample()
            timestep = 0.0
            set_initial_state = True
        elif model == "val":
            rbt, pbrv = setupValkyrieExample()
            timestep = 0.001
            # Setting initial state does not work for Timestepping RBT
            # in current configuration. It's probably something simple,
            # but I just want to hack the Val planar viz into working...
            set_initial_state = False
        else:
            print "Unrecognized model %s." % model
            parser.print_usage()
            exit(1)

        rbplant = RigidBodyPlant(rbt, timestep)

        builder = DiagramBuilder()
        rbplant_sys = builder.AddSystem(rbplant)

        torque = args.torque
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

        if set_initial_state:
            initial_state = np.zeros((rbt.get_num_positions() + rbt.get_num_velocities(), 1))
            initial_state[0] = 1.0
            state.SetFromVector(initial_state)

        simulator.StepTo(args.time)

        print(state.CopyToVector())