import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestControl(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(8)
    @timeout_decorator.timeout(1.)
    def test_lyapunov_controller(self):
        """Test Lyapunov controller"""
        lyapunov_controller = self.notebook_locals['lyapunov_controller']

        # Test some random points
        f_target = np.array([
            [-0.        ,  0.        ],
            [-0.98446505,  1.63115278],
            [-1.25328942, -0.5549188 ],
            [-0.94503173, -0.35599057],
            [-0.0477359 ,  1.58118168],
            [-0.68729272,  0.93799447],
            [-0.32667676,  1.04789874],
            [ 0.26038379, -1.93673728],
            [-0.48172147, -0.6600981 ],
            [-2.26737878, -1.3608832 ],
            [ 0.01895493,  1.67519335],
            [-0.09779214, -1.16880824],
            [ 1.95663546, -0.0321076 ],
            [-1.13915329,  0.34862014],
            [ 0.04540821, -1.67817773],
            [ 1.48942757,  0.69456878],
            [ 0.31220516,  0.47186241],
            [ 1.00893588, -1.49932112],
            [-0.14715458, -1.68457734],
            [ 0.0281199 ,  0.55990795],
            [-0.28278935, -1.27461434],
            [ 0.46212572, -1.53312378],
            [ 1.58881182, -0.60275863],
            [ 1.00400078, -1.14502099],
            [-0.0515139 ,  0.97757105],
            [-1.04841566, -0.34316574],
            [ 0.5732494 , -1.72569781],
            [ 0.12294264, -1.51026049],
            [-0.03111974, -1.56030365],
            [-0.56859115,  2.09971095],
            [ 0.08882972,  1.04781451],
            [ 0.39446391,  1.59797754],
            [-0.742823  ,  1.44942476],
            [-0.01041557,  2.01922169],
            [-0.0889171 , -1.26206768],
            [ 0.49202569, -1.23512984],
            [-1.42549336, -1.52190444],
            [ 0.56231803,  1.3662271 ],
            [ 0.30152939,  1.73773857],
            [ 0.15649151,  1.82285483],
            [ 0.09363587,  1.72416136],
            [-0.35980848, -0.45625562],
            [-1.31156436, -0.98129184],
            [ 0.34046179,  2.49985597],
            [-0.34964074,  0.35076956],
            [ 0.54254751, -1.08972773],
            [-0.66181161,  0.14943231],
            [-0.03146467, -1.62851599],
            [-0.35373363,  1.23793387],
            [-1.45732914, -0.89604197],
            [-1.00011818, -1.997144  ]
            ])
        np.random.seed(0)
        n_rands = 50
        tol = 0.1
        x_list = [np.zeros(3)]
        for i in range(n_rands):
            x = np.random.randn(3)
            if np.linalg.norm(x) <= tol:
                x *= tol / np.linlag.norm(x)
            x_list.append(x)
        f_eval = []
        for x in x_list:
            f_eval.append(lyapunov_controller(*x))
        self.assertLessEqual(
            np.linalg.norm(f_target - np.stack(f_eval)), 1e-5,
            'The state Lyapunov controller is not correct.')
