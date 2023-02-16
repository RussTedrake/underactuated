import unittest
import numpy as np
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
from underactuated import FindResource


class Testlpdp(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(6)
    @timeout_decorator.timeout(1.)
    def test_J_from_lp(self):
        """Test optimal cost-to-go from Linear Program"""
        # note: all prints here go to the output item in the json file
        J_value = self.notebook_locals['J_value']

        J = np.load(FindResource("exercises/dp/J.npy"))

        diff = np.abs(J - J_value)

        self.assertTrue(
            (diff <= 1e-6).all(),
            msg='|J - J*| > 1e-6',
        )
