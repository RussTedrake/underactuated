import typing

import numpy as np
from IPython.display import clear_output
from pydrake.all import (
    BatchEvalTimeDerivatives,
    Context,
    InputPortIndex,
    LeafSystem,
    MultilayerPerceptron,
    RandomGenerator,
    System,
)

from underactuated.optimizers import Adam


def ContinuousNeuralValueIteration(
    system: System,
    context: Context,
    value_mlp: MultilayerPerceptron,
    state_cost_function: typing.Callable[[np.ndarray], np.ndarray],
    R_diag: np.ndarray,
    state_samples: np.ndarray,
    time_step: float = 0.01,
    discount_factor: float = 1.0,
    input_port_index: InputPortIndex = 0,
    lr: float = 0.001,
    minibatch: int = None,
    epochs: int = 1000,
    optim_steps_per_epoch: int = 25,
    input_limits: np.ndarray = None,
    seed: int = 123,
):
    """
    Implements neural value iteration for a continuous-time system.

    It requires that the system has only continuous-time dynamics, and it
    assumes (currently without checking) that the system is control affine.

    Args:
        system : The system to be controlled.
        context : A context for the `system`.
    """
    input_port = system.get_input_port(input_port_index)
    num_states = system.num_continuous_states()
    num_inputs = input_port.size()
    N = state_samples.shape[1]

    # TODO(russt): why doesn't this binding work?
    # assert system.ValidateContext(context)
    assert context.has_only_continuous_state()
    assert value_mlp.get_input_port().size() == num_states
    assert value_mlp.layers()[-1] == 1
    assert R_diag.shape == (num_inputs,)
    assert state_samples.shape[0] == num_states
    assert time_step > 0.0
    assert discount_factor > 0.0 and discount_factor <= 1.0
    if input_limits != None:
        assert (
            num_inputs == 1
        ), "Input limits are only supported for scalar inputs (for now)"
        assert len(input_limits) == 2

    mlp_context = value_mlp.CreateDefaultContext()
    generator = RandomGenerator(seed)
    rng = np.random.default_rng(seed)
    value_mlp.SetRandomContext(mlp_context, generator)

    state_cost = state_cost_function(state_samples)
    Rinv = 1 / R_diag

    context.get_mutable_continuous_state_vector()

    # Precompute dynamics
    u = np.zeros((num_inputs, N))
    state_dynamics_x = BatchEvalTimeDerivatives(
        system, context, times=[0] * N, states=state_samples, inputs=u
    )
    dstate_dynamics_du = [np.empty((num_states, N))] * num_inputs
    for j in range(num_inputs):
        u[j] = 1
        dstate_dynamics_du[j] = BatchEvalTimeDerivatives(
            system, context, times=[0] * N, states=state_samples, inputs=u
        )
        u[j] = 0

    optimizer = Adam(value_mlp.GetMutableParameters(mlp_context), lr=lr)

    M = minibatch if minibatch else N
    J = np.zeros((1, M))
    Jnext = np.zeros((1, M))
    Jd = np.zeros((1, M))
    dJdX = np.asfortranarray(np.zeros((num_states, M)))
    dloss_dparams = np.zeros(value_mlp.num_parameters())
    last_loss = np.inf
    for epoch in range(epochs):
        if minibatch:
            batch = rng.integers(0, N, minibatch)
        else:
            batch = range(N)
        value_mlp.BatchOutput(mlp_context, state_samples[:, batch], J, dJdX)
        Xnext = state_samples[:, batch] + time_step * state_dynamics_x[:, batch]
        G = state_cost[batch]
        for i in range(num_inputs):
            ui = -0.5 * Rinv[i] * np.sum(dstate_dynamics_du[i][:, batch] * dJdX, 0)
            if input_limits != None:
                ui = np.minimum(np.maximum(ui, input_limits[0]), input_limits[1])
            G += R_diag[i] * ui**2
            Xnext += time_step * dstate_dynamics_du[i][:, batch] * ui
        value_mlp.BatchOutput(mlp_context, Xnext, Jnext)
        Jd[:] = G * time_step + discount_factor * Jnext
        for i in range(optim_steps_per_epoch):
            loss = value_mlp.BackpropagationMeanSquaredError(
                mlp_context, state_samples[:, batch], Jd, dloss_dparams
            )
            optimizer.step(loss, dloss_dparams)
        if not minibatch and np.linalg.norm(last_loss - loss) < 1e-8:
            break
        last_loss = loss
        if epoch % 20 == 0:
            clear_output(wait=True)
            print(f"epoch {epoch}: loss = {loss}")

    return mlp_context


class ContinuousNeuralValueIterationPolicy(LeafSystem):
    def __init__(
        self,
        system,
        value_mlp,
        value_mlp_context,
        R_diag,
        input_port_index=0,
        input_limits=None,
    ):
        LeafSystem.__init__(self)

        num_system_states = value_mlp.get_input_port().size()
        self._system = system
        self._context = system.CreateDefaultContext()

        self.value_mlp = value_mlp
        self.value_mlp_context = value_mlp_context
        self.J = np.zeros((1, 1))
        self.dJdX = np.asfortranarray(np.zeros((num_system_states, 1)))

        self.Rinv = 1 / R_diag
        self.input_limits = input_limits
        self.DeclareVectorInputPort("system_state", num_system_states)
        self._system_input_port = self._system.get_input_port(input_port_index)
        self.DeclareVectorOutputPort(
            "output", self._system_input_port.size(), self.CalcOutput
        )

    def CalcOutput(self, context, output):
        num_inputs = self._system_input_port.size()
        u = np.zeros(num_inputs)
        system_state = self.get_input_port().Eval(context)

        self.value_mlp.BatchOutput(
            self.value_mlp_context,
            np.atleast_2d(system_state).T,
            self.J,
            self.dJdX,
        )

        self._context.SetContinuousState(system_state)
        self._system_input_port.FixValue(self._context, u)
        state_dynamics_x = self._system.EvalTimeDerivatives(
            self._context
        ).CopyToVector()
        for i in range(num_inputs):
            u[i] = 1
            self._system_input_port.FixValue(self._context, u)
            dstate_dynamics_dui = (
                self._system.EvalTimeDerivatives(self._context).CopyToVector()
                - state_dynamics_x
            )
            ui = -0.5 * self.Rinv[i] * dstate_dynamics_dui.dot(self.dJdX)
            if self.input_limits != None:
                ui = np.minimum(
                    np.maximum(ui, self.input_limits[0]), self.input_limits[1]
                )
            output.SetAtIndex(i, ui)
            u[i] = 0
