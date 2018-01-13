
from pydrake import rbtree

robot = rbtree.RigidBodyTree("double_pendulum.urdf", rbtree.FloatingBaseType.kFixed);
print robot.get_position_name(0)
print robot.get_position_name(1)
