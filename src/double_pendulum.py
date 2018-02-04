
from pydrake.multibody.rigid_body_tree import *

robot = RigidBodyTree("double_pendulum.urdf", FloatingBaseType.kFixed);
print robot.get_position_name(0)
print robot.get_position_name(1)
