import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestControl(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(4)
    @timeout_decorator.timeout(1.)
    def test_lyapunov_controller_near_origin(self):
        """Test Lyapunov controller near origin"""
        # load controller from locals
        lyapunov_controller = self.notebook_locals['lyapunov_controller']

        # function that scales a vector in such a way that the third coordinate
        # has absolute value <= a desired tolerance
        # needed to handle the local linearization in the controller
        def project_in(x, tol):
            return x if np.abs(x[2]) <= tol else x * tol / np.abs(x[2])

        # generate random states at which evaluate the dynamics
        n = 50
        tol = 1e-4  # must be smaller than the one in the notebook (= 1e-3)
        np.random.seed(0)
        x = [project_in(np.random.randn(3), tol) for i in range(n)]

        # desired value of the dynamics at given test points
        f_des = np.array([[-1.80237445e-04, 2.40885019e-04],
                          [-2.29299489e-04, -8.90205042e-06],
                          [-9.20460164e-04, -3.46637175e-04],
                          [-2.82339256e-05, 2.09904847e-04],
                          [-1.71457707e-04, 2.27412727e-04],
                          [-1.62642401e-04, 5.28256832e-04],
                          [-1.22627869e-05, -2.33454726e-04],
                          [-8.80691725e-05, -8.35250685e-05],
                          [-4.96028881e-03, -2.97834965e-03],
                          [1.27391521e-05, 3.04316199e-04],
                          [-1.74532454e-05, -1.57403853e-04],
                          [1.26690727e-03, -2.25228291e-05],
                          [-3.17636327e-04, 1.10430312e-04],
                          [2.12886571e-05, -2.73840825e-04],
                          [3.34791107e-04, 1.82766025e-04],
                          [5.63446602e-05, 3.88667653e-05],
                          [1.80229828e-04, -2.23757479e-04],
                          [-3.27707902e-05, -2.43265392e-04],
                          [4.23683178e-05, 8.43941306e-04],
                          [-8.33850482e-05, -3.74869067e-04],
                          [8.26985819e-05, -2.44217525e-04],
                          [4.29657666e-04, -1.55840079e-04],
                          [1.79676102e-04, -1.48993377e-04],
                          [-4.02730836e-05, 7.65261365e-04],
                          [-2.83192333e-04, -1.06909774e-04],
                          [1.18305344e-04, -3.50435804e-04],
                          [2.67392673e-05, -1.95179561e-04],
                          [-5.86382545e-05, -1.69688244e-04],
                          [-1.26253646e-04, 3.60834920e-04],
                          [1.70633544e-05, 9.84540881e-05],
                          [1.93579156e-04, 7.86938038e-04],
                          [-1.38221925e-04, 2.50435880e-04],
                          [-8.27345956e-06, 1.60717126e-03],
                          [-2.98265004e-05, -6.02754193e-05],
                          [1.08299347e-04, -1.17366178e-04],
                          [-2.60031931e-04, -2.55336988e-04],
                          [-1.02965532e-04, 2.79275398e-04],
                          [-4.74352797e-05, 1.54911183e-04],
                          [2.82927224e-05, 2.84714144e-04],
                          [1.68086067e-05, 2.66588042e-04],
                          [-1.26216414e-04, -1.68631814e-04],
                          [-8.86416924e-04, -6.64176158e-04],
                          [6.47266017e-05, 4.75067403e-04],
                          [-7.55609542e-05, 5.72244360e-05],
                          [1.06052814e-04, -1.94993984e-04],
                          [-3.24741878e-04, 7.68095349e-05],
                          [-2.65552184e-05, -2.73297968e-04],
                          [-6.91921177e-05, 2.26246495e-04],
                          [-2.61074358e-04, -9.65319357e-05],
                          [-2.41996000e-04, -4.85084770e-04]])

        # test the function at the evaluation points
        f = np.vstack([lyapunov_controller(*xi) for xi in x])
        np.testing.assert_array_almost_equal(
            f,
            f_des,
            decimal=4,
            err_msg='The controller is not correct near the origin.')

    @weight(4)
    @timeout_decorator.timeout(1.)
    def test_lyapunov_controller_away_from_origin(self):
        """Test Lyapunov controller away from origin"""
        # load controller from locals
        lyapunov_controller = self.notebook_locals['lyapunov_controller']

        # function that scales a vector in such a way that the third coordinate
        # has absolute value >= a desired tolerance
        # needed to handle the local linearization in the controller
        def project_out(x, tol):
            return x if np.abs(x[2]) >= tol else x * tol / np.abs(x[2])

        # generate random states at which evaluate the dynamics
        n = 50
        tol = 1e-2  # must be greater than the one in the notebook (= 1e-3)
        np.random.seed(0)
        x = [project_out(np.random.randn(3), tol) for i in range(n)]

        # desired value of the dynamics at given test points
        f_des = np.array([[-0.98446505, 1.63115278], [-1.25328942, -0.5549188],
                          [-0.94503173, -0.35599057], [-0.0477359, 1.58118168],
                          [-0.68729272, 0.93799447], [-0.32667676, 1.04789874],
                          [0.26038379, -1.93673728], [-0.48172147, -0.6600981],
                          [-2.26737878, -1.3608832], [0.01895493, 1.67519335],
                          [-0.09779214, -1.16880824], [1.95663546, -0.0321076],
                          [-1.13915329, 0.34862014], [0.04540821, -1.67817773],
                          [1.48942757, 0.69456878], [0.31220516, 0.47186241],
                          [1.00893588, -1.49932112], [-0.14715458, -1.68457734],
                          [0.0281199, 0.55990795], [-0.28278935, -1.27461434],
                          [0.46212572, -1.53312378], [1.58881182, -0.60275863],
                          [1.00400078, -1.14502099], [-0.0515139, 0.97757105],
                          [-1.04841566, -0.34316574], [0.5732494, -1.72569781],
                          [0.12294264, -1.51026049], [-0.03111974, -1.56030365],
                          [-0.56859115, 2.09971095], [0.08882972, 1.04781451],
                          [0.39446391, 1.59797754], [-0.742823, 1.44942476],
                          [-0.01041557, 2.01922169], [-0.0889171, -1.26206768],
                          [0.49202569, -1.23512984], [-1.42549336, -1.52190444],
                          [0.56231803, 1.3662271], [0.30152939, 1.73773857],
                          [0.15649151, 1.82285483], [0.09363587, 1.72416136],
                          [-0.35980848, -0.45625562],
                          [-1.31156436, -0.98129184], [0.34046179, 2.49985597],
                          [-0.34964074, 0.35076956], [0.54254751, -1.08972773],
                          [-0.66181161, 0.14943231], [-0.03146467, -1.62851599],
                          [-0.35373363, 1.23793387], [-1.45732914, -0.89604197],
                          [-1.00011818, -1.997144]])

        # test the function at the evaluation points
        f = np.vstack([lyapunov_controller(*xi) for xi in x])
        np.testing.assert_array_almost_equal(
            f,
            f_des,
            decimal=4,
            err_msg='The controller is not correct far away from the origin.')
