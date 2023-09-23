from pydrake.multibody.tree import MultibodyForces_


def ManipulatorDynamics(plant, q, v=None, context=None):
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
