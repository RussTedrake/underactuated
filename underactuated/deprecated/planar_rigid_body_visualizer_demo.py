# -*- coding: utf8 -*-

import argparse
import math
import os.path

import numpy as np
import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import scipy as sp
import scipy.spatial

from pydrake.all import (
    AddModelInstanceFromUrdfStringSearchingInRosPackages,
    ConstantVectorSource,
    Context,
    DiagramBuilder,
    FloatingBaseType,
    PortDataType,
    SignalLogger,
    Simulator,
)
from pydrake.attic.multibody.rigid_body_tree import (
    RigidBodyFrame,
    RigidBodyTree,
)
from pydrake.attic.multibody.rigid_body_plant import RigidBodyPlant
from pydrake.attic.multibody.shapes import Shape
from pydrake.systems.pyplot_visualizer import PyPlotVisualizer

from underactuated import FindResource
from underactuated.deprecated.planar_rigid_body_visualizer import (
    PlanarRigidBodyVisualizer
)


def setupPendulumExample():
    rbt = RigidBodyTree(FindResource("pendulum/pendulum.urdf"),
                        floating_base_type=FloatingBaseType.kFixed)  # noqa
    Tview = np.array([[1., 0., 0., 0.],
                      [0., 0., 1., 0.],
                      [0., 0., 0., 1.]],
                     dtype=np.float64)
    pbrv = PlanarRigidBodyVisualizer(rbt, Tview, [-1.2, 1.2], [-1.2, 1.2])
    return rbt, pbrv


def setupDoublePendulumExample():
    rbt = RigidBodyTree(FindResource("double_pendulum/double_pendulum.urdf"),
                        floating_base_type=FloatingBaseType.kFixed)  # noqa
    Tview = np.array([[1., 0., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]],
                     dtype=np.float64)
    fig, ax = plt.subplots(1, 1)
    pbrv = PlanarRigidBodyVisualizer(rbt, Tview, [-2.5, 2.5], [-2.5, 2.5],
                                     use_random_colors=True, ax=ax)
    return rbt, pbrv


def setupValkyrieExample():
    # Valkyrie Example
    rbt = RigidBodyTree()
    world_frame = RigidBodyFrame("world_frame", rbt.world(),
                                 [0, 0, 0], [0, 0, 0])
    from pydrake.multibody.parsers import PackageMap
    import pydrake
    pmap = PackageMap()
    # Note: Val model is currently not installed in drake binary distribution.
    pmap.PopulateFromFolder(os.path.join(pydrake.getDrakePath(), "examples"))
    # TODO(russt): remove plane.urdf and call AddFlatTerrainTOWorld instead
    AddModelInstanceFromUrdfStringSearchingInRosPackages(
        open(FindResource("deprecated/plane.urdf"), 'r').read(),
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
    pbrv = PlanarRigidBodyVisualizer(rbt, Tview, [-2.0, 2.0], [-0.25, 3.0],
                                     use_random_colors=True)
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
            print("Unrecognized model %s." % model)
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

        simulator.AdvanceTo(args.duration)

        print(state.CopyToVector())

        # Generate an animation of whatever happened
        ani = visualizer.animate(signalLogger, repeat=True)

        if (args.animate):
            print("Animating the simulation on repeat -- "
                  "close the plot to continue.")
            plt.show()
