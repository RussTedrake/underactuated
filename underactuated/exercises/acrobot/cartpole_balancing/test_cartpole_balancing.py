import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestCartPoleBalancing(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(4)
    @timeout_decorator.timeout(1.)
    def test_q1_f(self):
        """Test state space dynamics"""
        # note: all prints here go to the output item in the json file
        f = self.notebook_locals['f']

        # Test some random points
        f_target = np.array(
            [[4.38409231e-01, 7.23465178e-01, 4.18245285e+00, -9.87220594e+00],
             [7.20511334e-02, 2.68438980e-01, 3.79255211e+00, -8.03904598e+00],
             [3.80941133e-01, 6.59363469e-02, 3.42017412e+00, -9.43643712e+00],
             [4.52123962e-01, 9.31206020e-01, 2.14278108e+00, -4.17164251e+00],
             [2.30302879e-01, 5.48489919e-01, 3.48728183e+00, -1.00084705e+01],
             [7.50409859e-01, 6.69013241e-01, 3.95123851e+00, -8.32565877e+00],
             [3.72384689e-01, 4.77401155e-01, 3.72390310e+00, -7.90784674e+00],
             [3.13994677e-01, 5.72625333e-01, 3.64485538e+00, -9.43966511e+00],
             [6.57399463e-01, 3.70351083e-01, 3.29492846e+00, -6.48304604e+00],
             [9.06423269e-01, 1.80451619e-01, 3.75547418e+00, -7.37698681e+00],
             [6.34379869e-01, 5.22906201e-01, 3.60550623e+00, -7.34044503e+00],
             [7.09394394e-01, 5.24345597e-01, 1.60780955e+00, -2.50478137e+00],
             [5.31286907e-02, 3.08852685e-01, 3.90143941e+00, -9.21715191e+00],
             [9.45048224e-01, 8.48400881e-01, 3.37526553e+00, -9.98615149e+00],
             [3.08733657e-01, 4.62996394e-01, 2.00707420e+00, -3.27236171e+00],
             [3.43536530e-01, 3.24426170e-01, 1.61083575e+00, -2.93433562e+00],
             [4.48120658e-01, 7.74900376e-01, 4.00635261e+00, -7.62084869e+00],
             [7.78213602e-01, 8.87288952e-01, 4.11746726e+00, -8.04898448e+00],
             [4.06558094e-02, 8.75671725e-01, 3.37335289e+00, -9.90898347e+00],
             [7.17242232e-01, 1.47147572e-01, 3.69068823e+00, -9.59497774e+00],
             [8.12829550e-01, 4.27704833e-01, 3.45438694e+00, -6.66540106e+00],
             [7.60515120e-01, 7.14327528e-03, 3.45924611e+00, -9.53761546e+00],
             [5.41442132e-01, 6.07770751e-01, 1.38800524e+00, -1.93003879e+00],
             [2.30430674e-01, 6.59158397e-01, 1.40841086e+00, -2.65055484e+00],
             [1.69523724e-01, 7.82230151e-01, 4.37333299e+00, -9.00428216e+00],
             [7.96951357e-01, 9.75139681e-01, 4.01364732e+00, -8.43927548e+00],
             [9.09182464e-01, 1.97532889e-01, 3.32850312e+00, -9.66949809e+00],
             [1.68275305e-01, 6.64968963e-01, 3.85206601e+00, -9.89558565e+00],
             [3.55288048e-02, 2.81533818e-01, 2.34462851e+00, -3.92146710e+00],
             [3.61616648e-01, 6.36222857e-02, 2.30104314e-01, -3.10699554e-01],
             [6.96695902e-01, 4.27053493e-01, 3.58063917e+00, -8.01343660e+00],
             [9.40661393e-01, 9.92557719e-01, 4.06645825e+00, -8.83892346e+00],
             [9.26612938e-01, 4.58603480e-01, 3.76115740e+00, -9.77988079e+00],
             [8.72627183e-01, 2.39030261e-02, 3.67287849e+00, -8.62800990e+00],
             [9.10713454e-01, 3.04392667e-02, 1.81829719e+00, -2.98561915e+00],
             [4.18099543e-01, 1.81404289e-01, 3.35463608e+00, -6.60164495e+00],
             [7.37088060e-01, 1.60202411e-01, 2.83126077e+00, -5.74851118e+00],
             [2.06285857e-01, 9.18709205e-01, 3.96622992e+00, -7.30961283e+00],
             [2.32671096e-01, 4.51078603e-01, 3.23194637e+00, -6.55657744e+00],
             [3.82511139e-01, 6.50128329e-01, 3.45709437e+00, -9.90750669e+00],
             [7.44823674e-01, 9.46275927e-01, 1.25737131e+00, -1.85953499e+00],
             [7.12048787e-01, 6.56450287e-01, 3.74336560e+00, -9.03868842e+00],
             [4.24625533e-01, 5.93637329e-01, 2.97087147e+00, -9.72528129e+00],
             [6.36502342e-01, 7.61084744e-01, 3.59033828e+00, -9.61683315e+00],
             [5.96049120e-01, 9.19048522e-02, 3.03744306e+00, -5.33844587e+00],
             [1.04711304e-01, 6.84904037e-01, 4.07469516e+00, -7.89932265e+00],
             [7.85420722e-01, 1.45679433e-01, 2.94576525e+00, -5.21104260e+00],
             [5.13243400e-01, 6.28851403e-01, 2.82436669e+00, -5.50623306e+00],
             [8.92337457e-01, 8.35468561e-01, 3.46592464e+00, -7.17428002e+00],
             [7.54053197e-01, 5.42631657e-01, 3.11107307e+00, -5.95955488e+00]])
        np.random.seed(7)
        n_rands = 50
        f_eval = []
        for i in range(n_rands):
            x = np.random.rand(4)
            u = np.random.rand(1)
            f_eval.append(f(x, u))
        self.assertLessEqual(
            np.linalg.norm(f_target - np.stack(f_eval)), 1e-5,
            'The state space dynamics f(x,u) are not correct.')

    @weight(2)
    @timeout_decorator.timeout(1.)
    def test_q2_get_A_lin(self):
        """Test linearization matrix A"""
        # note: all prints here go to the output item in the json file
        get_A_lin = self.notebook_locals['get_A_lin']
        A_lin = get_A_lin()
        g = 9.81
        A_lin_true = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [0, g, 0, 0],
                               [0, 2 * g, 0, 0]])

        err = np.linalg.norm(np.abs(A_lin_true - A_lin))
        self.assertLessEqual(err, 1e-8, "incorrect A linearization")
        # a = hash(tuple(np.ndarray.flatten(A_lin)))
        # self.assertTrue(np.allclose(A_lin_true, A_lin))
        # self.assertEqual(a, -4269322539335713771,
        #                  "Incorrect linearization matrix A.")

    @weight(2)
    @timeout_decorator.timeout(1.)
    def test_q3_get_B_lin(self):
        """Test linearization matrix B"""
        # note: all prints here go to the output item in the json file
        get_B_lin = self.notebook_locals['get_B_lin']
        B_lin = get_B_lin()

        B_lin_true = np.array([[0], [0], [1], [1]])

        err = np.linalg.norm(np.abs(B_lin_true - B_lin))
        self.assertLessEqual(err, 1e-8, "incorrect B linearization")

        # a = hash(tuple(np.ndarray.flatten(B_lin)))
        # self.assertEqual(a, 4686582722430018711,
        #                  "Incorrect linearization matrix B.")

    @weight(1)
    @timeout_decorator.timeout(1.)
    def test_q4_x_and_u_list(self):
        """Test x_list and u_list content"""
        # note: all prints here go to the output item in the json file
        x_list = self.notebook_locals['x_list']
        x_list_target = [
            np.array([0, .99 * np.pi, 0, 0]),
            np.array([0, .9 * np.pi, 0, 0]),
            np.array([0, .85 * np.pi, 0, 0]),
            np.array([0, .5 * np.pi, 0, 0]),
            np.array([0, 0, 0, 0]),
            np.array([1, np.pi, 0, 0])
        ]
        diff = np.linalg.norm(np.stack(x_list) - np.stack(x_list_target))
        self.assertLessEqual(diff, 1e-7, "Incorrect x_list.")

        u_list = self.notebook_locals['u_list']
        u_list_target = [
            np.array([0]),
            np.array([-10]),
            np.array([0]),
            np.array([0]),
            np.array([0]),
            np.array([10])
        ]
        diff = np.linalg.norm(np.stack(u_list) - np.stack(u_list_target))
        self.assertLessEqual(diff, 1e-7, "Incorrect u_list.")

    @weight(2)
    @timeout_decorator.timeout(1.)
    def test_q5_linearization_errors(self):
        """Test linearization errors"""
        # note: all prints here go to the output item in the json file
        errors = self.notebook_locals['errors']
        linearization_error = self.notebook_locals['linearization_error']
        x_list = self.notebook_locals['x_list']
        u_list = self.notebook_locals['u_list']
        for i in range(len(u_list)):
            self.assertLess(
                np.linalg.norm(errors[i]
                               - [linearization_error(x_list[i], u_list[i])]),
                0.00001, 'Unable to reproduce errors')

        ub = [0.002, 2.4, 2.3, 26.1, 70, 0.001]
        for i in range(len(u_list)):
            self.assertLess(errors[i], ub[i],
                            'Some linearization errors are too high')

        lb = [0.00001, 1.0, 1.0, 23, 60, 0.00]
        for i in range(len(u_list)):
            self.assertLessEqual(lb[i], errors[i],
                                 'Some linearization errors are too low')

    @weight(2)
    @timeout_decorator.timeout(1.)
    def test_q6_system_recovers_from_states(self):
        """Test for which the system is able to recover"""
        # note: all prints here go to the output item in the json file
        system_recovers_from_states = \
            self.notebook_locals['system_recovers_from_states']
        arr = np.sort(np.asarray(system_recovers_from_states))
        true_recover = np.array([0, 1, 2, 5])
        same = np.all(arr == true_recover)
        self.assertTrue(same)
