from pydrake.multibody.rigid_body_tree import RigidBodyTree
from manipulator_dynamics import *
import underactuated_utils as utils

tree = RigidBodyTree(utils.findResource("double_pendulum.urdf"), FloatingBaseType.kFixed);

q = (1., 1.)
v = (0.1, 0.1)
(M, Cv, tauG, B) = manipulator_dynamics(tree,q,v)
print("M = " + str(M))
print("Cv = " + str(Cv))
print("tauG = " + str(tauG))
print("B = " + str(B))