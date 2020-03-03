import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestVanDerPol(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(5)
    @timeout_decorator.timeout(1.)
    def test_rho_method_1(self):
        """Test level set for Method 1"""
        rho_method_1 = self.notebook_locals['rho_method_1']
        self.assertEqual(
            hash(self.round(rho_method_1, 1)),
            691752902764107778,
            "Error in the implementation of Method 1."
            )

    @weight(5)
    @timeout_decorator.timeout(1.)
    def test_rho_method_2(self):
        """Test level set for Method 2"""
        rho_method_2 = self.notebook_locals['rho_method_2']
        self.assertEqual(
            hash(self.round(rho_method_2, 4)),
            702129196305569794,
            "Error in the implementation of Method 2."
            )

    @weight(5)
    @timeout_decorator.timeout(1.)
    def test_rho_method_3(self):
        """Test level set for Method 3"""
        rho_method_3 = self.notebook_locals['rho_method_3']
        self.assertEqual(
            hash(self.round(rho_method_3, 4)*2),
            1404258392611139588,
            "Error in the implementation of Method 3."
            )

    @staticmethod
    def round(n, decimals=0):
        multiplier = 10 ** decimals
        return round(n * multiplier) / multiplier
