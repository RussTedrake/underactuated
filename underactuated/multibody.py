import numpy as np
import numpy.typing as npt
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import JointActuatorIndex, MultibodyForces_
from pydrake.systems.framework import Context


def ManipulatorDynamics(
    plant: MultibodyPlant,
    q: npt.NDArray,
    v: npt.NDArray = None,
    context: Context = None,
):
    """Returns the components of the manipulator equations -- M(q), C(q,v), tauG(q), B(q), and tauExt(q, v) -- for a given MultibodyPlant.

    Args:
        plant: The MultibodyPlant for which to compute the manipulator equations.
        q: The generalized positions.
        v: The generalized velocities.  If None, the velocities are taken from the
           context.
        context: The Context to use for the computation.  If None, a new default
                 Context will be created.
    """
    if context is None:
        context = plant.CreateDefaultContext()
    plant.SetPositions(context, q)
    if v is not None:
        plant.SetVelocities(context, v)
    M = plant.CalcMassMatrixViaInverseDynamics(context)
    Cv = plant.CalcBiasTerm(context)
    tauG = plant.CalcGravityGeneralizedForces(context)
    B = plant.MakeActuationMatrix()
    forces = MultibodyForces_(plant)
    plant.CalcForceElementsContribution(context, forces)
    # TODO(russt): add in contact forces to tauExt.
    tauExt = plant.CalcGeneralizedForces(context, forces) - tauG

    return (M, Cv, tauG, B, tauExt)


def MakePidStateProjectionMatrix(plant: MultibodyPlant):
    """Given a MultibodyPlant, returns a selection matrix, S, such that [q_a; v_a] = S @ [q; v], where q_a and v_a are the actuated positions and velocities and q, v, are the full state. This can be passed to e.g. the `state_projection` argument of the PidController constructor.

    Args:
        plant: The MultibodyPlant for the projection matrix.
    """
    num_q = plant.num_positions()
    num_v = plant.num_velocities()
    num_u = plant.num_actuators()
    S = np.zeros((2 * num_u, num_q + num_v))
    j = 0
    for i in range(plant.num_actuators()):
        actuator = plant.get_joint_actuator(JointActuatorIndex(i))
        assert actuator.num_inputs() == 1
        joint = actuator.joint()
        S[j, joint.position_start()] = 1
        S[num_u + j, num_q + joint.velocity_start()] = 1
        j = j + 1
    return S
