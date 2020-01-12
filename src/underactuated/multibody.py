
from pydrake.attic.multibody.rigid_body_tree import RigidBodyTree
from pydrake.autodiffutils import AutoDiffXd
from pydrake.multibody.tree import MultibodyForces_
from pydrake.multibody.plant import MultibodyPlant_
from pydrake.symbolic import Expression
import numpy as np

def ManipulatorDynamics(plant, q, v=None):
    if isinstance(plant, RigidBodyTree):
        # TODO(russt): Zap this branch once I'm finished porting to MBP.
        if v is None:
            v = np.zeros((plant.get_num_velocities(),1))
        kinsol = plant.doKinematics(q,v)
        M = plant.massMatrix(kinsol)
        tauG = -plant.dynamicsBiasTerm(kinsol, {}, False)
        Cv = plant.dynamicsBiasTerm(kinsol, {}, True) + tauG
        B = plant.B
        tauExt = 0*tauG

    else:
        # MultibodyPlant version
        T = None
        for scalar in [float, AutoDiffXd, Expression]:
            if isinstance(plant, MultibodyPlant_[scalar]):
                T = scalar
        assert(T)

        context = plant.CreateDefaultContext()
        plant.SetPositions(context, q)
        if v is not None:
            plant.SetVelocities(context, v)
        M = plant.CalcMassMatrixViaInverseDynamics(context)
        Cv = plant.CalcBiasTerm(context)
        tauG = plant.CalcGravityGeneralizedForces(context)
        B = plant.MakeActuationMatrix()
        forces = MultibodyForces_[T](plant)
        plant.CalcForceElementsContribution(context, forces)
        tauExt = forces.generalized_forces()

    return (M, Cv, tauG, B, tauExt)
