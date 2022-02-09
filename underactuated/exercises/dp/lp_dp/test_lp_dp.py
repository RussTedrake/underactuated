import unittest
import numpy as np
import cvxpy as cp
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight


class Testlpdp(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(8)
    @timeout_decorator.timeout(1.)
    def test_J_from_lp(self):
        """Test optimal cost-to-go from Linear Program"""
        # note: all prints here go to the output item in the json file
        J_value = self.notebook_locals['J_value']
        state_dim = self.notebook_locals['state_dim']
        input_dim = self.notebook_locals['input_dim']
        states = self.notebook_locals['states']
        inputs = self.notebook_locals['inputs']
        min_time_cost = self.notebook_locals['min_time_cost']
        T = self.notebook_locals['T']
        gamma = self.notebook_locals['gamma']

        J = cp.Variable(state_dim)

        constraints = []

        for i in range(input_dim):
            l = np.zeros(state_dim)
            for j in range(state_dim):
                l[j] = min_time_cost(states[:, j], inputs[:, i])
            constraints += [J <= l + gamma * T[:, :, i] @ J]
        c = np.random.rand(state_dim)

        objective = cp.Maximize(c @ J)
        problem = cp.Problem(objective, constraints)
        problem.solve()

        J = np.reshape(J.value, (21,21))

        diff = np.abs(J - J_value)

        self.assertTrue(
            (diff <= 1e-6).all(),
            msg='|J - J*| > 1e-6',
        )

