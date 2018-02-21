import math
import numpy as np

from pydrake.all import (DiagramBuilder, FloatingBaseType,
                         LinearQuadraticRegulator, RigidBodyTree, Simulator)
from pydrake.examples.acrobot import (AcrobotInput, AcrobotPlant, AcrobotState)
from underactuated import (FindResource, PlanarRigidBodyVisualizer)


def UprightState():
    state = AcrobotState()
    state.set_theta1(math.pi)
    state.set_theta2(0.)
    state.set_theta1dot(0.)
    state.set_theta2dot(0.)
    return state

def BalancingLQR():
    # Design an LQR controller for stabilizing the Acrobot around the upright.
    # Returns a (static) AffineSystem that implements the controller (in
    # the original AcrobotState coordinates).

    acrobot = AcrobotPlant()
    context = acrobot.CreateDefaultContext()

    input = AcrobotInput()
    input.set_tau(0.)
    context.FixInputPort(0, input)

    context.get_mutable_continuous_state_vector().SetFromVector(UprightState().CopyToVector())

    Q = np.diag((10.,10.,1.,1.))
    R = [1]

    return LinearQuadraticRegulator(acrobot, context, Q, R)

if __name__ == "__main__":
    builder = DiagramBuilder()

    acrobot = builder.AddSystem(AcrobotPlant())
    controller = builder.AddSystem(BalancingLQR())
    builder.Connect(acrobot.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), acrobot.get_input_port(0))

    tree = RigidBodyTree(FindResource("acrobot/acrobot.urdf"),
                         FloatingBaseType.kFixed)
    visualizer = builder.AddSystem(PlanarRigidBodyVisualizer(tree,
                                                             xlim=[-4., 4.],
                                                             ylim=[-4., 4.]))
    builder.Connect(acrobot.get_output_port(0), visualizer.get_input_port(0))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)
    context = simulator.get_mutable_context()

    state = context.get_mutable_continuous_state_vector()

    for i in range(5):
        context.set_time(0.)
        state.SetFromVector(UprightState().CopyToVector() + 0.1*np.random.randn(4,))
        simulator.StepTo(10.)
