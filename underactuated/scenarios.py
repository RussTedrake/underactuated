"""
This file contains a number of helper utilities to set up our various
experiments with less code.
"""
import numpy as np
from pydrake.all import (Box, CoulombFriction, Cylinder, RigidTransform,
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
