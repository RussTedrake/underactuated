import numpy as np
import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight

from pydrake.all import System


class TestDrakeSystems(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(1)
    @timeout_decorator.timeout(5.)
    def test_input_and_state(self):
        # note: all prints here go to the output item in the json file

        system = self.notebook_locals['pendulum_system']

        assert isinstance(system, System), "pendulum_system is not a System"
        assert system.num_input_ports() == 1, "should have 1 input port"
        assert system.get_input_port(
            0).size() == 1, "the input port is the wrong size"
        assert system.num_continuous_states(
        ) == 2, "wrong size for the state vector"
        assert system.num_discrete_state_groups(
        ) == 0, "your system should not have discrete states"

    @weight(1)
    @timeout_decorator.timeout(5.)
    def test_dynamics(self):
        # note: all prints here go to the output item in the json file

        system = self.notebook_locals['pendulum_system']
        context = system.CreateDefaultContext()

        q = np.linspace(0, 1, 5)
        v = np.linspace(0, 1, 5)
        u = np.linspace(0, 1, 5)
        qs, vs, us = np.meshgrid(q, v, u)
        for qi, vi, ui in zip(qs.flat, vs.flat, us.flat):
            context.SetContinuousState([qi, vi])
            system.get_input_port(0).FixValue(context, [ui])
            xdot = [vi, ui - vi - 10 * np.sin(qi)]
            np.testing.assert_almost_equal(
                xdot,
                system.EvalTimeDerivatives(context).CopyToVector())
