import unittest

import numpy as np
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
from pydrake.all import AffineSystem, Diagram, System


class TestDrakeDiagrams(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(1)
    @timeout_decorator.timeout(5.)
    def test_actuator_model(self):
        # note: all prints here go to the output item in the json file

        actuator_model = self.notebook_locals['actuator_model']

        assert isinstance(actuator_model,
                          System), "actuator_model must be a System"
        assert actuator_model.ToAutoDiffXdMaybe(
        ), "Your system isn't compatible with Drake's AutoDiffXd; see the note above."  # noqa
        assert actuator_model.num_input_ports(
        ) == 1, "must have exactly one input port"
        assert actuator_model.get_input_port().size(
        ) == 1, "input port must have size 1"
        assert actuator_model.num_output_ports(
        ) == 1, "must have only one output port"
        assert actuator_model.get_output_port().size(
        ) == 1, "output port should have size 1"
        assert actuator_model.num_continuous_states(
        ) == 1, "should have exactly one continuous state variable"
        assert actuator_model.num_discrete_state_groups(
        ) == 0, "should not have any discrete state"

        context = actuator_model.CreateDefaultContext()

        def check_dynamics(x, u):
            context.SetContinuousState([x])
            actuator_model.get_input_port().FixValue(context, [u])
            xdot = actuator_model.EvalTimeDerivatives(context).CopyToVector()
            self.assertAlmostEqual(
                2 * (u - x), xdot[0],
                'the state dynamics (xdot) of your actuator model are incorrect'
            )
            y = actuator_model.get_output_port().Eval(context)
            self.assertAlmostEqual(
                y, x, 'the output value of your actuator model is incorrect')

        check_dynamics(x=0, u=0)
        check_dynamics(x=0.4, u=0.1)
        check_dynamics(x=0.52, u=0.25)
        check_dynamics(x=-0.91, u=5.2)
        check_dynamics(x=2, u=-2)

    @weight(1)
    @timeout_decorator.timeout(5.)
    def test_diagram(self):
        diagram = self.notebook_locals['diagram']
        assert isinstance(diagram, Diagram), 'diagram must be a Diagram'
        assert len(diagram.GetSystems()
                  ) == 2, 'diagram should have exactly two subsystems'  # noqa
        assert diagram.ToAutoDiffXdMaybe(
        ), "Your system isn't compatible with Drake's AutoDiffXd; see the note above."  # noqa
        assert diagram.num_input_ports(
        ) == 1, "diagram should have exactly one input port"
        assert diagram.get_input_port().size(
        ) == 1, "diagram's input port should have size 1"
        assert diagram.num_continuous_states(
        ) == 3, "diagram should have a total of 3 continuous states"
        assert diagram.num_discrete_state_groups(
        ) == 0, "diagram should not have any discrete states"

    @weight(1)
    @timeout_decorator.timeout(5.)
    def test_diagram_context(self):
        diagram = self.notebook_locals['diagram']
        double_integrator = self.notebook_locals['double_integrator']
        actuator_model = self.notebook_locals['actuator_model']
        context = self.notebook_locals['diagram_context']

        double_integrator_context = double_integrator.GetMyContextFromRoot(
            context)
        np.testing.assert_almost_equal(
            double_integrator_context.get_continuous_state_vector().get_value(),
            [1.2, 0],
            err_msg="the value of the double integrator state is incorrect")
        actuator_model_context = actuator_model.GetMyContextFromRoot(context)
        np.testing.assert_almost_equal(
            actuator_model_context.get_continuous_state_vector().get_value(),
            [0],
            err_msg="the value of the actuator model state is incorrect")

        assert diagram.get_input_port().HasValue(
            context), "you must fix a value for the diagram's input port"
        np.testing.assert_almost_equal(
            diagram.get_input_port().Eval(context), [0],
            err_msg="the input port value is incorrect")

    @weight(1)
    @timeout_decorator.timeout(5.)
    def test_controller(self):
        controller = self.notebook_locals['controller']
        assert isinstance(controller,
                          AffineSystem), "controller should be an affine system"

        assert controller.num_continuous_states(
        ) == 0, "your controller has state; that shouldn't happen"
        np.testing.assert_almost_equal(
            np.sort(controller.D()),
            np.sort([[-1.06519664, -1., -2.26503715]]),
            err_msg="your LQR gains do not match the expected value")
