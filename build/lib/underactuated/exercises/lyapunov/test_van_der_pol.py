import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np
from pydrake.all import Solve


class TestVanDerPol(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(5)
    @timeout_decorator.timeout(5.)
    def test_method_1(self):
        """Test level set for Method 1"""
        # test level set value up to first digit
        rho_method_1 = self.notebook_locals['rho_method_1']
        self.assertEqual(hash(self.round(rho_method_1, 1)), 691752902764107778,
                         "The level set from Method 1 is incorrect.")

        # test that the line search converged
        is_verified = self.notebook_locals['is_verified']
        self.assertTrue(
            is_verified(rho_method_1),
            "Could not verify the level set rho_method_1 with the function"
            + "is_verified(rho).")
        rho_step = self.notebook_locals['rho_step']
        self.assertAlmostEqual(
            rho_step,
            0.01,
            msg="rho_step is different from the predefined value of 0.01.")
        self.assertFalse(
            is_verified(rho_method_1 + rho_step),
            "The function is_verified(rho) could verify the level set"
            + "rho_method_1 + rho_step: line search is not complete.")

    @weight(5)
    @timeout_decorator.timeout(5.)
    def test_method_2(self):
        """Test level set for Method 2"""
        # test level set value up to fourth digit
        # (note that the degree of the polynomial does not affect precision
        # in this particular problem)
        rho_method_2 = self.notebook_locals['rho_method_2']
        self.assertEqual(hash(self.round(rho_method_2, 4)), 702129196305569794,
                         "The level set from Method 2 is incorrect.")

        # try to solve again the optimization problem
        # and test that the optimal value of rho is the same
        # to retrieve rho, find the only optimization variable in the cost
        prog = self.notebook_locals['prog2']
        costs = prog.GetAllCosts()
        self.assertEqual(len(costs), 1,
                         "prog2 has more than one cost function.")
        variables = prog.GetAllCosts()[0].variables()
        self.assertEqual(len(variables), 1,
                         "The cost function of prog2 is incorrect.")
        rho = variables[0]
        result = Solve(prog)
        self.assertAlmostEqual(
            result.GetSolution(rho),
            rho_method_2,
            msg="Solving prog2 we got a different value of rho_method_2.")

    @weight(5)
    @timeout_decorator.timeout(5.)
    def test_method_3(self):
        """Test level set for Method 3"""
        # test level set value up to fourth digit
        # (note that the degree of the polynomial does not affect precision
        # in this particular problem)
        rho_method_3 = self.notebook_locals['rho_method_3']
        self.assertEqual(hash(self.round(rho_method_3, 4) * 2),
                         1404258392611139588,
                         "The level set from Method 3 is incorrect.")

        # try to solve again the optimization problem
        # and test that the optimal value of rho is the same
        # to retrieve rho, find the only optimization variable in the cost
        prog = self.notebook_locals['prog3']
        costs = prog.GetAllCosts()
        self.assertEqual(len(costs), 1,
                         "prog3 has more than one cost function.")
        variables = prog.GetAllCosts()[0].variables()
        self.assertEqual(len(variables), 1,
                         "The cost function of prog3 is incorrect.")
        rho = variables[0]
        result = Solve(prog)
        self.assertAlmostEqual(
            result.GetSolution(rho),
            rho_method_3,
            msg="Solving prog3 we got a different value of rho_method_3.")

    @staticmethod
    def round(n, decimals=0):
        multiplier = 10**decimals
        return round(n * multiplier) / multiplier
