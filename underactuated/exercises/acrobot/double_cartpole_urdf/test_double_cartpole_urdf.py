import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestDoubleCartPoleURDF(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(3)
    @timeout_decorator.timeout(1.)
    def test_q1_get_single_pendulum_urdf_parameters(self):
        """Test the single_pendulum_urdf_parameters"""
        # note: all prints here go to the output item in the json file
        get_single_pendulum_urdf_parameters = self.notebook_locals[
            'get_single_pendulum_urdf_parameters']
        urdf_parameters = get_single_pendulum_urdf_parameters()
        a = 0
        for x in get_single_pendulum_urdf_parameters():
            a += hash(tuple(x))
        self.assertEqual(a, -3713082714464628219,
                         "Incorrect single pendulum urdf parameters.")

    @weight(5)
    @timeout_decorator.timeout(1.)
    def test_q2_get_double_pendulum_urdf_parameters(self):
        """Test the double_pendulum_urdf_parameters"""
        # note: all prints here go to the output item in the json file
        get_double_pendulum_urdf_parameters = self.notebook_locals[
            'get_double_pendulum_urdf_parameters']
        urdf_parameters = get_double_pendulum_urdf_parameters()
        a = 0
        for x in get_double_pendulum_urdf_parameters():
            a += hash(tuple(x))
        self.assertEqual(a, -7426164346401008863,
                         "Incorrect double pendulum urdf parameters.")
