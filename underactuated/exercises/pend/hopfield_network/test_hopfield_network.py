import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestHopfield(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(6)
    @timeout_decorator.timeout(5.)
    def test_Hopfield_Network(self):
        """Test Hopfield Network"""
        # note: all prints here go to the output item in the json file

        dynamics = self.notebook_locals['dynamics']
        calc_A = self.notebook_locals['calculate_A']

        A = np.array([[1, 0], [0, 1], [-1, -1]])
        beta = 5

        x = np.array([0, 0])
        x_target = np.array([0, 0])
        x_nxt = dynamics(x, A, beta)

        self.assertTrue(
            all(abs(i - j) <= 1e-4 for i, j in zip(x_nxt, x_target)),
            msg='For x = [0, 0], x_dot should be {}'.format(x_target),
        )

        x = np.array([1, 0])
        x_target = np.array([-0.00678274, 0.00664746])
        x_nxt = dynamics(x, A, beta)

        self.assertTrue(
            all(abs(i - j) <= 1e-4 for i, j in zip(x_nxt, x_target)),
            msg='For x = [1, 0], x_dot should be {}'.format(x_target),
        )

        x = np.array([1, 1])
        x_target = np.array([-0.50000023, -0.50000023])
        x_nxt = dynamics(x, A, beta)

        self.assertTrue(
            all(abs(i - j) <= 1e-4 for i, j in zip(x_nxt, x_target)),
            msg='Answer is incorrect for x = [1, 1].',
        )

        A = []
        for item in self.notebook_locals['training_data']:
            A += [item.reshape((-1))]
        A_target = np.array(A)
        A_calc = calc_A(self.notebook_locals['training_data'])

        self.assertTrue(
            all(
                abs(i - j) <= 1e-4
                for i, j in zip(A_target.shape, A_calc.shape)),
            msg='Expecting the shape of A to be {}, but got {}.'.format(
                A_target.shape, A_calc.shape),
        )

        A_calc = calc_A(
            [np.zeros((28, 28)).reshape((-1)),
             np.ones((28, 28)).reshape((-1))])
        A_target1 = np.array(
            [np.zeros((28, 28)).reshape((-1)),
             np.ones((28, 28)).reshape((-1))])
        A_target2 = np.array(
            [np.ones((28, 28)).reshape((-1)),
             np.zeros((28, 28)).reshape((-1))])

        diff_a = (abs(A_target1 - A_calc)).max() < 1e-4
        diff_b = (abs(A_target2 - A_calc)).max() < 1e-4
        self.assertTrue(diff_a or diff_b,
                        msg='calculate_A did not produce expected output.')
