import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np
from pydrake.all import (DiagramBuilder, SceneGraph, Simulator, LeafSystem,
                         DiagramBuilder, SceneGraph, ZeroOrderHold)
from pydrake.examples import PendulumGeometry, PendulumPlant


class TestPendulumCVI(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(2)
    @timeout_decorator.timeout(20.)
    def test_state_cost(self):
        """Test compute state cost"""
        # note: all prints here go to the output item in the json file

        # the loss must be close to the correct value
        compute_state_cost = self.notebook_locals['compute_state_cost']
        tol = 1e-8

        # intentionally different values than for cartpole
        target_state = np.array([0, np.pi]).reshape(-1, 1)
        Q = np.diag([1., 20.])

        self.assertLessEqual(
            compute_state_cost(Q, target_state, target_state),
            tol,
            msg=('target state cost is '
                 f'{compute_state_cost(Q, target_state, target_state)}'
                 ' which is more than tolerance {tol}'))

        cost = compute_state_cost(Q, target_state,
                                  np.array([0, 0]).reshape(-1, 1))
        target = np.pi**2 * 20.
        self.assertLessEqual(np.abs(cost - target),
                             tol,
                             msg='Cost at state = 0 is incorrect')

        state1 = [
            5.89375203, 5.72132364, 2.07924145, -25.58778621, 4.84281921,
            -19.20542969, -2.35612669
        ]
        state2 = [
            -35.69553553, 24.11618679, -0.85857409, 3.61505439, -3.28412933,
            26.96593774, -0.81457745
        ]
        test_states = np.array([state1, state2])
        result_true = np.array([
            30201.18682396, 8831.40552794, 324.34992454, 659.21812345,
            849.25095811, 11720.8369055, 318.57697075
        ])
        result_tested = compute_state_cost(Q, target_state, test_states)
        for i in range(result_true.shape[0]):
            self.assertLessEqual(np.abs(result_true[i] - result_tested[i]), tol,
                                 f"failed on random state number {i}")

    @weight(2)
    @timeout_decorator.timeout(20.)
    def test_compute_u_star(self):
        """Test compute u_star"""
        # note: all prints here go to the output item in the json file
        compute_u_star = self.notebook_locals['compute_u_star']

        R_diag = np.array([1.5])

        dJdX = np.array(
            [[8.455e+00, -2.330e+00, 1.650e-01, 2.040e+00, -3.945e+00],
             [1.000e-02, -5.000e-03, -8.775e+00, 5.090e+00, 3.000e+00]])
        dstate_dynamics_du = np.array([[[-0.785, 3.86, -6.79, 3.51, 5.265]],
                                       [[1.35, -4.855, 5.24, 6.095, -0.26]]])
        u_star_true = np.array(
            [[2.20789167, 2.98984167, 15.70045, -12.72798333, 7.183475]])
        u_star_test = compute_u_star(R_diag, dJdX, dstate_dynamics_du)
        self.assertTrue(u_star_true.shape == u_star_test.shape,
                        (f"output of compute_u_star is incorrect. Expected "
                         f"{u_star_true.shape} got {u_star_test.shape}"))

        for i in range(u_star_test.shape[0]):
            for j in range(u_star_test.shape[1]):
                self.assertLessEqual(
                    np.abs(u_star_test[i, j] - u_star_true[i, j]), 1e-8,
                    f"incorrect u_star at index {i} for test state {j}")

    @weight(6)
    @timeout_decorator.timeout(600.)
    def test_policy(self):
        """Test policy"""
        value_mlp = self.notebook_locals['value_mlp']
        input_limits = self.notebook_locals['input_limits']
        value_mlp_context = self.notebook_locals['value_mlp_context']
        R_diag = self.notebook_locals['R_diag']
        compute_u_star = self.notebook_locals['compute_u_star']
        simulator = self.build_pendulum_simulator(value_mlp,
                                                  value_mlp_context,
                                                  R_diag,
                                                  compute_u_star,
                                                  input_limits=input_limits)
        simulator_context = simulator.get_mutable_context()
        simulator.set_target_realtime_rate(0.)
        num_sim = 20
        np.random.seed(14)
        errors = []
        for i in range(num_sim):
            duration = 5.0
            simulator_context.SetTime(0.)
            simulator_context.SetContinuousState(
                np.array([2 * np.pi * np.random.rand(), 0]))
            simulator.Initialize()
            simulator.AdvanceTo(duration)
            final_state_vector = simulator_context.get_continuous_state(
            ).get_vector().CopyToVector()
            theta_wrapped = np.arccos(np.cos(final_state_vector[0]))
            errors.append(np.abs(theta_wrapped - np.pi))

        num_passed = 0
        for e in errors:
            # check if theta is close to pi
            if np.abs(e) <= 1e-1:
                num_passed += 1
        frac = 0.7
        threshold = int(frac * num_sim)
        self.assertGreaterEqual(
            num_passed, threshold,
            (f"Only passed {num_passed/num_sim * 100}% of initial"
             f" configurations. Need to pass {frac*100}%"))

    # initialize controller and plant
    def build_pendulum_simulator(self,
                                 value_mlp,
                                 value_mlp_context,
                                 R_diag,
                                 compute_u_star,
                                 input_limits=None):
        time_step = 0.01
        closed_loop_builder = DiagramBuilder()
        plant_cl, scene_graph_cl = closed_loop_builder.AddSystem(
            PendulumPlant()), closed_loop_builder.AddSystem(SceneGraph())

        controller_sys = ContinuousFittedValueIterationPolicyComputeUStar(
            plant_cl,
            value_mlp,
            value_mlp_context,
            R_diag,
            compute_u_star,
            input_limits=input_limits)

        PendulumGeometry.AddToBuilder(closed_loop_builder,
                                      plant_cl.get_state_output_port(),
                                      scene_graph_cl)

        controller = closed_loop_builder.AddSystem(controller_sys)
        # we assume a zero-order hold between time steps
        zoh = closed_loop_builder.AddSystem(ZeroOrderHold(time_step, 1))

        # wire all the systems together
        closed_loop_builder.Connect(plant_cl.get_output_port(),
                                    controller.get_input_port())
        closed_loop_builder.Connect(controller.get_output_port(),
                                    zoh.get_input_port())
        closed_loop_builder.Connect(zoh.get_output_port(),
                                    plant_cl.get_input_port())

        diagram_closed_loop = closed_loop_builder.Build()

        simulator = Simulator(diagram_closed_loop)
        return simulator


class ContinuousFittedValueIterationPolicyComputeUStar(LeafSystem):

    def __init__(self,
                 plant,
                 value_mlp,
                 value_mlp_context,
                 R_diag,
                 compute_u_star,
                 input_port_index=0,
                 input_limits=None):
        LeafSystem.__init__(self)

        self.num_plant_states = value_mlp.get_input_port().size()
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()

        self.value_mlp = value_mlp
        self.value_mlp_context = value_mlp_context
        self.J = np.zeros((1, 1))
        self.dJdX = np.asfortranarray(np.zeros((self.num_plant_states, 1)))

        self.compute_u_star = compute_u_star

        self.Rinv = 1 / R_diag
        self.R_diag = R_diag
        self.input_limits = input_limits
        self.DeclareVectorInputPort("plant_state", self.num_plant_states)
        self._plant_input_port = self._plant.get_input_port(input_port_index)
        self.DeclareVectorOutputPort("output", self._plant_input_port.size(),
                                     self.CalcOutput)

    def CalcOutput(self, context, output):
        num_inputs = self._plant_input_port.size()
        u = np.zeros(num_inputs)
        plant_state = self.get_input_port().Eval(context)

        self.value_mlp.BatchOutput(self.value_mlp_context,
                                   np.atleast_2d(plant_state).T, self.J,
                                   self.dJdX)

        self._plant_context.SetContinuousState(plant_state)
        self._plant_input_port.FixValue(self._plant_context, u)
        state_dynamics_x = self._plant.EvalTimeDerivatives(
            self._plant_context).CopyToVector()

        dstate_dynamics_du = np.empty((self.num_plant_states, num_inputs, 1))
        for i in range(num_inputs):
            u[i] = 1
            self._plant_input_port.FixValue(self._plant_context, u)
            dstate_dynamics_du[:, :, i] = (self._plant.EvalTimeDerivatives(
                self._plant_context).CopyToVector() - state_dynamics_x).reshape(
                    -1, 1)
            u[i] = 0

        u_star = self.compute_u_star(self.R_diag, self.dJdX,
                                     dstate_dynamics_du)[:, 0]
        if self.input_limits is not None:
            u_star = np.clip(u_star, self.input_limits[0], self.input_limits[1])
        for i in range(num_inputs):
            output.SetAtIndex(i, u_star[i])
