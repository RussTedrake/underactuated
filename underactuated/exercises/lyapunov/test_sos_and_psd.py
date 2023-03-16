import unittest

import numpy as np
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight


class TestSOSandPSD(unittest.TestCase):
    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(2)
    @timeout_decorator.timeout(1.0)
    def test_sos(self):
        """Test SOS functions"""
        # load the response from locals
        obtained = self.notebook_locals["is_function_sos"]
        assert len(obtained) == 3, "is_functions_sos must have length 3"

        # multiplying arbitrary value to mask the difference with the target
        err = np.linalg.norm(obtained - np.array([0.0, 1.0, 0.0])) * 123.45
        self.assertLessEqual(err, 1e-6, "incorrect SOS results")

    @weight(2)
    @timeout_decorator.timeout(1.0)
    def test_psd(self):
        """Test PSD functions"""
        # load the response from locals
        obtained = self.notebook_locals["is_function_psd"]
        assert len(obtained) == 3, "is_functions_psd must have length 3"

        # multiplying arbitrary value to mask the difference with the target
        err = np.linalg.norm(obtained - np.array([1.0, 1.0, 1.0])) * 123.45
        self.assertLessEqual(err, 1e-6, "incorrect PSD results")
