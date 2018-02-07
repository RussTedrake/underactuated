
from pydrake.multibody.rigid_body_tree import *
import numpy as np

def ManipulatorDynamics(tree, q, v=None):
    if v is None:
        v = np.zeros((tree.get_num_velocities(),1))
    kinsol = tree.doKinematics(q,v)
    M = tree.massMatrix(kinsol)
    tauG = -tree.dynamicsBiasTerm(kinsol, {}, False)
    Cv = tree.dynamicsBiasTerm(kinsol, {}, True) + tauG
    B = tree.B

    return (M, Cv, tauG, B)
