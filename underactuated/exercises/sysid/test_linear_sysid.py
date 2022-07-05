import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestLinearSysid(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(3)
    @timeout_decorator.timeout(20.)
    def test_L2_regression(self):
        """Test L2 regression"""
        Ahat = self.notebook_locals['Ahat_2norm']
        Bhat = self.notebook_locals['Bhat_2norm']
        X = self.notebook_locals['X']
        U = self.notebook_locals['U']
        res = Ahat @ X[:, :-1] + Bhat @ U[:, :-1] - X[:, 1:]
        self.assertLessEqual(
            np.sum(res**2), 6e-7,
            'The residual for 2-norm minimization is too high!')

    @weight(3)
    @timeout_decorator.timeout(20.)
    def test_Linf_regression(self):
        """Test L infinite regression"""
        Ahat = self.notebook_locals['Ahat_infnorm']
        Bhat = self.notebook_locals['Bhat_infnorm']
        X = self.notebook_locals['X']
        U = self.notebook_locals['U']
        res = Ahat @ X[:, :-1] + Bhat @ U[:, :-1] - X[:, 1:]
        residual = np.sum(np.max(np.abs(res), axis=0))
        self.assertLessEqual(
            residual, 0.02,
            'The residual for infinity-norm minimization is too high!')

        obj_infnorm = self.notebook_locals['obj_infnorm']
        self.assertLessEqual(
            np.abs(obj_infnorm - residual), 0.1,
            'Objective from optimizer and empirical evaluations are different!')

    @weight(3)
    @timeout_decorator.timeout(20.)
    def test_L1_regression(self):
        """Test L1 regression"""
        Ahat = self.notebook_locals['Ahat_1norm']
        Bhat = self.notebook_locals['Bhat_1norm']
        X = self.notebook_locals['X']
        U = self.notebook_locals['U']
        res = Ahat @ X[:, :-1] + Bhat @ U[:, :-1] - X[:, 1:]
        residual = np.sum(np.abs(res))
        self.assertLessEqual(
            residual, 0.04, 'The residual for 1-norm minimization is too high!')

        obj_1norm = self.notebook_locals['obj_1norm']
        self.assertLessEqual(
            np.abs(obj_1norm - residual), 0.1,
            'Objective from optimizer and empirical evaluations are different!')
