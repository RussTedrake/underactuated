import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight


class TestGliderSysid(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(3)
    @timeout_decorator.timeout(20.)
    def test_lq_fit(self):
        """Test least squares fitting"""
        # note: all prints here go to the output item in the json file

        # the best config and best score should have the right values
        best_config = self.notebook_locals['best_config']
        best_score = self.notebook_locals['best_score']
        self.assertFalse(best_config != 6, 'The chosen basis set is incorrect')
        self.assertFalse(best_score != 63831, 'The best score is incorrect')

    @weight(3)
    @timeout_decorator.timeout(20.)
    def test_basis_dynamics(self):
        """Test basis function dynamics"""
        # note: all prints here go to the output item in the json file
        trajectories_fit = self.notebook_locals['trajectories_fit']
        last_x = trajectories_fit[0][-1][0]
        last_z = trajectories_fit[0][-1][1]
        last_th = trajectories_fit[0][-1][2]
        self.assertLessEqual(last_x,
                             3.36,
                             msg='Simulated trajectory x is incorrect')
        self.assertGreaterEqual(last_x,
                                3.34,
                                msg='Simulated trajectory x is incorrect')
        self.assertLessEqual(last_z,
                             5.10,
                             msg='Simulated trajectory z is incorrect')
        self.assertGreaterEqual(last_z,
                                4.99,
                                msg='Simulated trajectory z is incorrect')
        self.assertLessEqual(last_th,
                             -0.52,
                             msg='Simulated trajectory theta is incorrect')
        self.assertGreaterEqual(last_th,
                                -0.54,
                                msg='Simulated trajectory theta is incorrect')
