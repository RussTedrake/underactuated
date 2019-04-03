# -*- coding: utf8 -*-

'''

Usage:

MeshcatRigidBodyVisualizer acts identically to PyPlotVisualizer
from the perspective of the code: it is a System block that consumes
system state and visualizes that state at every publish step.

However, where PyPlotVisualizer opens its own visualizer window,
MeshcatRigidBodyVisualizer expects the command `meshcat-server`
to already be running in another terminal. The visualization
will be available at the web url provided by that server in real
time (either when the system is being simulated, or when the
animate() method is called).

'''

import argparse
import math
import os.path
import time

import numpy as np

import pydrake
from pydrake.all import (
    AddModelInstanceFromUrdfStringSearchingInRosPackages,
    ConstantVectorSource,
    Context,
    DiagramBuilder,
    FloatingBaseType,
    LeafSystem,
    PiecewisePolynomial,
    PortDataType,
    RigidBodyFrame,
    RigidBodyPlant,
    RigidBodyTree,
    Shape,
    SignalLogger,
    Simulator,
)

from underactuated.utils import FindResource, Rgba2Hex

import meshcat
import meshcat.transformations as tf


class MeshcatRigidBodyVisualizer(LeafSystem):
    def __init__(self,
                 rbtree,
                 draw_timestep=0.033333,
                 prefix="RBViz",
                 zmq_url="tcp://127.0.0.1:6000",
                 draw_collision=False):
        LeafSystem.__init__(self)
        self.set_name('meshcat_visualization')
        self.timestep = draw_timestep
        self.DeclarePeriodicPublish(draw_timestep, 0.0)
        self.rbtree = rbtree
        self.draw_collision = draw_collision

        self.DeclareInputPort(PortDataType.kVectorValued,
                              self.rbtree.get_num_positions() +
                              self.rbtree.get_num_velocities())

        # Set up meshcat
        self.prefix = prefix
        self.vis = meshcat.Visualizer(zmq_url=zmq_url)
        self.vis[self.prefix].delete()

        # Publish the tree geometry to get the visualizer started
        self.PublishAllGeometry()

    def PublishAllGeometry(self):
        n_bodies = self.rbtree.get_num_bodies()-1
        all_meshcat_geometry = {}
        for body_i in range(n_bodies):

            body = self.rbtree.get_body(body_i+1)
            # TODO(gizatt) Replace these body-unique indices
            # with more readable body.get_model_name() or other
            # model index information when an appropriate
            # function gets bound in pydrake.
            body_name = body.get_name() + ("(%d)" % body_i)

            if self.draw_collision:
                draw_elements = [self.rbtree.FindCollisionElement(k)
                                 for k in body.get_collision_element_ids()]
            else:
                draw_elements = body.get_visual_elements()

            for element_i, element in enumerate(draw_elements):
                element_local_tf = element.getLocalTransform()
                if element.hasGeometry():
                    geom = element.getGeometry()

                    geom_type = geom.getShape()
                    if geom_type == Shape.SPHERE:
                        meshcat_geom = meshcat.geometry.Sphere(geom.radius)
                    elif geom_type == Shape.BOX:
                        meshcat_geom = meshcat.geometry.Box(geom.size)
                    elif geom_type == Shape.CYLINDER:
                        meshcat_geom = meshcat.geometry.Cylinder(
                            geom.length, geom.radius)
                        # In Drake, cylinders are along +z
                        # In meshcat, cylinders are along +y
                        # Rotate to fix this misalignment
                        extra_rotation = tf.rotation_matrix(
                            math.pi/2., [1, 0, 0])
                        element_local_tf[0:3, 0:3] = \
                            element_local_tf[0:3, 0:3].dot(
                                extra_rotation[0:3, 0:3])
                    elif geom_type == Shape.MESH:
                        meshcat_geom = \
                            meshcat.geometry.ObjMeshGeometry.from_file(
                                geom.resolved_filename[0:-3] + "obj")
                        # respect mesh scale
                        element_local_tf[0:3, 0:3] *= geom.scale
                    else:
                        print "UNSUPPORTED GEOMETRY TYPE ",\
                              geom.getShape(), " IGNORED"
                        continue

                    rgba = [1., 0.7, 0., 1.]
                    if not self.draw_collision:
                        rgba = element.getMaterial()
                    self.vis[self.prefix][body_name][str(element_i)]\
                        .set_object(meshcat_geom,
                                    meshcat.geometry.MeshLambertMaterial(
                                        color=Rgba2Hex(rgba)))
                    self.vis[self.prefix][body_name][str(element_i)].\
                        set_transform(element_local_tf)

    def DoPublish(self, context, event):
        self.draw(context)

    def draw(self, context):
        ''' Evaluates the robot state and draws it.
            Can be passed either a raw state vector, or
            an input context.'''

        if isinstance(context, Context):
            positions = self.EvalVectorInput(context, 0).get_value()[0:self.rbtree.get_num_positions()]  # noqa
        else:
            positions = context[0:self.rbtree.get_num_positions()]

        kinsol = self.rbtree.doKinematics(positions)

        body_fill_index = 0
        for body_i in range(self.rbtree.get_num_bodies()-1):
            tf = self.rbtree.relativeTransform(kinsol, 0, body_i+1)
            body = self.rbtree.get_body(body_i+1)
            # Don't try to update the transform of geometry
            # that doesn't exist.
            if ((self.draw_collision and
                    len(body.get_collision_element_ids()) > 0)
                or
                (not self.draw_collision and
                    len(body.get_visual_elements()) > 0)):
                body_name = body.get_name() + ("(%d)" % body_i)
                self.vis[self.prefix][body_name].set_transform(tf)

    def animate(self, log, resample=True, time_scaling=1.0):
        # log - a reference to a pydrake.systems.primitives.SignalLogger that
        # contains the plant state after running a simulation.
        # resample -- should we do a resampling operation to make
        # the samples more consistent in time? This can be disabled
        # if you know the draw_timestep passed into the constructor exactly
        # matches the sample timestep of the log.

        if type(log) is SignalLogger:
            t = log.sample_times()
            x = log.data()

            if resample:
                import scipy.interpolate

                t_resample = np.arange(0, t[-1], self.timestep)
                x = scipy.interpolate.interp1d(t, x, kind='linear', axis=1)(t_resample)  # noqa
                t = t_resample

        # TODO(russt): Replace PiecewisePolynomial with Trajectory if I ever
        # add the pydrake bindings for the base class.
        elif type(log) is PiecewisePolynomial:
            t = np.arange(log.start_time(), log.end_time(), self.timestep)
            x = np.hstack([log.value(dt) for dt in t])

        def animate_update(i):
            self.draw(x[:, i])

        # Keep track of real elapsed vs sim elapsed time when playing back
        # animation, and sleep whenever we get ahead of the simulation
        # time (scaled by the time scaling factor).
        start_time = time.time()
        sim_start_time = t[0]
        for i in range(t.shape[0]):
            animate_update(i)
            sim_time = t[i] - sim_start_time
            real_time = time.time() - start_time
            time.sleep(max(0, sim_time/time_scaling - real_time))


def setupPendulumExample():
    rbt = RigidBodyTree(FindResource("pendulum/pendulum.urdf"),
                        floating_base_type=FloatingBaseType.kFixed)  # noqa
    Tview = np.array([[1., 0., 0., 0.],
                      [0., 0., 1., 0.],
                      [0., 0., 0., 1.]],
                     dtype=np.float64)
    pbrv = MeshcatRigidBodyVisualizer(rbt)
    return rbt, pbrv


def setupDoublePendulumExample():
    rbt = RigidBodyTree(FindResource("double_pendulum/double_pendulum.urdf"),
                        floating_base_type=FloatingBaseType.kFixed)  # noqa
    Tview = np.array([[1., 0., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]],
                     dtype=np.float64)
    pbrv = MeshcatRigidBodyVisualizer(rbt)
    return rbt, pbrv


def setupValkyrieExample():
    # Valkyrie Example
    rbt = RigidBodyTree()
    world_frame = RigidBodyFrame("world_frame", rbt.world(),
                                 [0, 0, 0], [0, 0, 0])
    from pydrake.multibody.parsers import PackageMap
    pmap = PackageMap()
    # Note: Val model is currently not installed in drake binary distribution.
    pmap.PopulateFromFolder(os.path.join(pydrake. getDrakePath(), "examples"))
    # TODO(russt): remove plane.urdf and call AddFlatTerrainTOWorld instead
    AddModelInstanceFromUrdfStringSearchingInRosPackages(
        open(FindResource(os.path.join("underactuated", "plane.urdf")), 'r').read(),  # noqa
        pmap,
        pydrake.getDrakePath() + "/examples/",
        FloatingBaseType.kFixed,
        world_frame,
        rbt)
    val_start_frame = RigidBodyFrame("val_start_frame", rbt.world(),
                                     [0, 0, 1.5], [0, 0, 0])
    AddModelInstanceFromUrdfStringSearchingInRosPackages(
        open(pydrake.getDrakePath() + "/examples/valkyrie/urdf/urdf/valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf", 'r').read(),  # noqa
        pmap,
        pydrake.getDrakePath() + "/examples/",
        FloatingBaseType.kRollPitchYaw,
        val_start_frame,
        rbt)
    Tview = np.array([[1., 0., 0., 0.],
                      [0., 0., 1., 0.],
                      [0., 0., 0., 1.]],
                     dtype=np.float64)
    pbrv = MeshcatRigidBodyVisualizer(rbt, draw_collision=True)
    return rbt, pbrv


if __name__ == "__main__":

    # Usage demo: load a URDF, rig it up with a constant torque input, and
    # draw it.
    np.set_printoptions(precision=5, suppress=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--torque",
                        type=float,
                        help="Constant torque to apply to all joints.",
                        default=1.0)
    parser.add_argument("-T", "--duration",
                        type=float,
                        help="Duration to run sim.",
                        default=1.0)
    parser.add_argument("-m", "--models",
                        type=str,
                        nargs="*",
                        help="Models to run, at least one of [pend, dpend, "
                             "val]",
                        default=["dpend"])
    parser.add_argument("-a", "--animate",
                        action="store_true",
                        help="Enable real-time looping animation after each "
                             "simulation.")
    parser.add_argument("--test",
                        action="store_true",
                        help="Help out CI by launching a meshcat server for "
                             "the duration of the test.")
    args = parser.parse_args()

    meshcat_server_p = None
    if args.test:
        print "Spawning"
        import subprocess
        meshcat_server_p = subprocess.Popen(["meshcat-server"])
    else:
        print "Warning: if you have not yet run meshcat-server in another " \
              "terminal, this will hang."

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
        nx = rbt.get_num_positions() + rbt.get_num_velocities()

        builder = DiagramBuilder()
        rbplant_sys = builder.AddSystem(rbplant)

        torque = args.torque
        torque_system = builder.AddSystem(ConstantVectorSource(
                                np.ones((rbt.get_num_actuators(), 1))*torque))
        builder.Connect(torque_system.get_output_port(0),
                        rbplant_sys.get_input_port(0))
        print('Simulating with constant torque = '
              + str(torque) + ' Newton-meters')

        # Visualize
        visualizer = builder.AddSystem(pbrv)
        builder.Connect(rbplant_sys.get_output_port(0),
                        visualizer.get_input_port(0))

        # And also log
        signalLogRate = 60
        signalLogger = builder.AddSystem(SignalLogger(nx))
        signalLogger.DeclarePeriodicPublish(1. / signalLogRate, 0.0)
        builder.Connect(rbplant_sys.get_output_port(0),
                        signalLogger.get_input_port(0))

        diagram = builder.Build()
        simulator = Simulator(diagram)
        simulator.Initialize()
        simulator.set_target_realtime_rate(1.0)
        simulator.set_publish_every_time_step(False)

        # TODO(russt): Clean up state vector access below.
        state = simulator.get_mutable_context().get_mutable_state()\
                         .get_mutable_continuous_state().get_mutable_vector()

        if set_initial_state:
            initial_state = np.zeros((nx, 1))
            initial_state[0] = 1.0
            state.SetFromVector(initial_state)

        simulator.StepTo(args.duration)

        if (args.animate):
            # Generate an animation of whatever happened
            ani = visualizer.animate(signalLogger)

    if meshcat_server_p is not None:
        meshcat_server_p.kill()
        meshcat_server_p.wait()
