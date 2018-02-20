from pydrake.all import (FloatingBaseType, RigidBodyTree)
from underactuated import (FindResource, ManipulatorDynamics)

tree = RigidBodyTree(FindResource("double_pendulum/double_pendulum.urdf"),
                     FloatingBaseType.kFixed)

q = (1., 1.)
v = (0.1, 0.1)
(M, Cv, tauG, B) = ManipulatorDynamics(tree, q, v)
print("M = " + str(M))
print("Cv = " + str(Cv))
print("tauG = " + str(tauG))
print("B = " + str(B))
