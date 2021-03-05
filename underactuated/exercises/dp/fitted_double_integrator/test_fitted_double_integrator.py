import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight


class TestFittedDoubleIntegrator(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(2)
    @timeout_decorator.timeout(20.)
    def test_final_losses(self):
        """Test model training"""
        # note: all prints here go to the output item in the json file

        # the loss must be close to the correct value
        final_loss_min_time = self.notebook_locals['final_loss_min_time']
        self.assertLessEqual(
            final_loss_min_time,
            0.0003,
            msg='Min time loss is < than upper bound for the correct value')
        self.assertGreaterEqual(
            final_loss_min_time,
            0.0001,
            msg='Min time loss is > than lower bound for the correct value')

        final_loss_quadratic = self.notebook_locals['final_loss_quadratic']
        self.assertLessEqual(
            final_loss_quadratic,
            0.009,
            msg='Quadratic loss is < than upper bound for the correct value')
        self.assertGreaterEqual(
            final_loss_quadratic,
            0.002,
            msg='Quadratic loss is > than lower bound for the correct value')

    @weight(2)
    @timeout_decorator.timeout(20.)
    def test_policy(self):
        """Test closed form policy"""
        # note: all prints here go to the output item in the json file
        final_state_min_time = self.notebook_locals['final_state_min_time']
        self.assertLessEqual(
            final_state_min_time[0],
            0.6,
            msg='Minimum time policy final position is too large')
        self.assertGreaterEqual(
            final_state_min_time[0],
            -0.6,
            msg='Minimum time policy final position is too small')
        self.assertLessEqual(
            final_state_min_time[1],
            0.5,
            msg='Minimum time policy final velocity is too large')
        self.assertGreaterEqual(
            final_state_min_time[1],
            -0.5,
            msg='Minimum time policy final velocity is too small')

        final_state_quadratic = self.notebook_locals['final_state_quadratic']
        self.assertLessEqual(final_state_quadratic[0],
                             0.6,
                             msg='Quadratic policy final position is too large')
        self.assertGreaterEqual(
            final_state_quadratic[0],
            -0.6,
            msg='Quadratic policy final position is too small')
        self.assertLessEqual(final_state_quadratic[1],
                             0.5,
                             msg='Quadratic policy final velocity is too large')
        self.assertGreaterEqual(
            final_state_quadratic[1],
            -0.5,
            msg='Quadratic policy final velocity is too small')
