"""
This file contains a number of helper utilities to set up our various
experiments with less code.
"""
import numpy as np
from pydrake.all import (BallRpyJoint, Box, CoulombFriction, Cylinder,
                         PrismaticJoint, RevoluteJoint, RigidTransform,
                         SpatialInertia, Sphere, UnitInertia)


def AddShape(plant, shape, name, mass=1, mu=1, color=[.5, .5, .9, 1.0]):
    instance = plant.AddModelInstance(name)
    # TODO: Add a method to UnitInertia that accepts a geometry shape (unless
    # that dependency is somehow gross) and does this.
    if isinstance(shape, Box):
        inertia = UnitInertia.SolidBox(shape.width(), shape.depth(),
                                       shape.height())
    elif isinstance(shape, Cylinder):
        inertia = UnitInertia.SolidCylinder(shape.radius(), shape.length())
    elif isinstance(shape, Sphere):
        inertia = UnitInertia.SolidSphere(shape.radius())
    else:
        raise RunTimeError(
            f"need to write the unit inertia for shapes of type {shape}")
    body = plant.AddRigidBody(
        name, instance,
        SpatialInertia(mass=mass,
                       p_PScm_E=np.array([0., 0., 0.]),
                       G_SP_E=inertia))
    if plant.geometry_source_is_registered():
        if isinstance(shape, Box):
            plant.RegisterCollisionGeometry(
                body, RigidTransform(),
                Box(shape.width() - 0.001,
                    shape.depth() - 0.001,
                    shape.height() - 0.001), name, CoulombFriction(mu, mu))
            i = 0
            for x in [-shape.width() / 2.0, shape.width() / 2.0]:
                for y in [-shape.depth() / 2.0, shape.depth() / 2.0]:
                    for z in [-shape.height() / 2.0, shape.height() / 2.0]:
                        plant.RegisterCollisionGeometry(
                            body, RigidTransform([x, y, z]),
                            Sphere(radius=1e-7), f"contact_sphere{i}",
                            CoulombFriction(mu, mu))
                        i += 1
        else:
            plant.RegisterCollisionGeometry(body, RigidTransform(), shape, name,
                                            CoulombFriction(mu, mu))

        plant.RegisterVisualGeometry(body, RigidTransform(), shape, name, color)

    return instance


# https://github.com/RobotLocomotion/drake/issues/14949
def AddFloatingRpyJoint(plant, frame, instance, use_ball_rpy=True):
    inertia = SpatialInertia(mass=0,
                             p_PScm_E=[0., 0., 0.],
                             G_SP_E=UnitInertia(0, 0, 0))
    x_body = plant.AddRigidBody("x", instance, inertia)
    plant.AddJoint(
        PrismaticJoint("x", plant.world_frame(), x_body.body_frame(),
                       [1, 0, 0]))
    y_body = plant.AddRigidBody("y", instance, inertia)
    plant.AddJoint(
        PrismaticJoint("y", x_body.body_frame(), y_body.body_frame(),
                       [0, 1, 0]))
    z_body = plant.AddRigidBody("z", instance, inertia)
    plant.AddJoint(
        PrismaticJoint("z", y_body.body_frame(), z_body.body_frame(),
                       [0, 0, 1]))
    if use_ball_rpy:
        plant.AddJoint(BallRpyJoint("ball", z_body.body_frame(), frame))
    else:
        # RollPitchYaw is body z-y-x
        rz_body = plant.AddRigidBody("rz", instance, inertia)
        plant.AddJoint(
            RevoluteJoint("rz", z_body.body_frame(), rz_body.body_frame(),
                          [0, 0, 1]))
        ry_body = plant.AddRigidBody("ry", instance, inertia)
        plant.AddJoint(
            RevoluteJoint("ry", rz_body.body_frame(), ry_body.body_frame(),
                          [0, 1, 0]))
        plant.AddJoint(
            RevoluteJoint("rx", ry_body.body_frame(), frame, [1, 0, 0]))
