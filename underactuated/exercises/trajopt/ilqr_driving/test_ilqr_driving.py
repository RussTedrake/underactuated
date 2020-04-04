import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestIlqrDriving(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(1)
    @timeout_decorator.timeout(1.)
    def test1_discrete_dynamics(self):
        """Test discrete_dynamics"""
        # load locals
        f = self.notebook_locals['discrete_dynamics']

        f_target = [
            np.array(
                [0.14181286, 0.81062987, 0.54580749, 0.77731476, 1.02810156]),
            np.array([0.131663, 0.300996, 0.57034388, 0.71732411, 0.81033267]),
            np.array(
                [0.33233256, 0.91916814, 0.27416228, 0.45461388, 0.99126091]),
            np.array(
                [1.02770659, 0.27770476, 0.56066881, 0.96146963, 0.20821043]),
            np.array([0.71706372, 0.47773599, 0.22401885, 0.538506, 0.40897373])
        ]

        np.random.seed(7)
        n_rands = 5
        f_eval = []
        for i in range(n_rands):
            x = np.random.rand(5)
            u = np.random.rand(2)
            f_eval = f(x, u)
            np.testing.assert_array_almost_equal(
                f_eval,
                f_target[i],
                err_msg='The discrete dynamics are incorrect. '
                f'Expected output {f_target[i]} with x={x} and u={u}')

    @weight(1)
    @timeout_decorator.timeout(1.)
    def test2_rollout(self):
        """Test rollout"""
        # load locals
        f = self.notebook_locals['rollout']

        f_target_hash = [
            5548026545464297818, 5303601661710898095, -8752618797308043950,
            5303655996741254866, 1117140169305939251
        ]

        np.random.seed(7)
        n_rands = 5
        for i in range(n_rands):
            N = 5
            n_u = 2
            x0 = np.array([1, 0, 0, 1, 0])
            u_trj = np.random.randn(N - 1, n_u) * 0.4
            f_eval_hash = hash(
                tuple(np.ndarray.flatten(self.round(f(x0, u_trj), 4))))
            self.assertEqual(f_eval_hash, f_target_hash[i], "Incorrect rollout")

    @weight(1)
    @timeout_decorator.timeout(1.)
    def test3_cost_trj(self):
        """Test cost_trj"""
        # load locals
        f = self.notebook_locals['cost_trj']

        f_target = [
            29.946731429035616, 34.805336401404986, 27.626872505986434,
            33.25730851334137, 33.83897480138296
        ]

        np.random.seed(7)
        n_rands = 5
        f_eval = []
        for i in range(n_rands):
            N = 5
            n_u = 2
            n_x = 5
            x0 = np.array([1, 0, 0, 1, 0])
            u_trj = np.random.randn(N - 1, n_u) * 0.4
            x_trj = np.random.randn(N, n_x) * 0.4
            f_eval = f(x_trj, u_trj)
            np.testing.assert_almost_equal(
                f_eval,
                f_target[i],
                err_msg='The cost_trj function is incorrect. '
                f'Expected output {f_target[i]} with x_trj=\n{x_trj}'
                f' and u=\n{u_trj}')

    @weight(5)
    @timeout_decorator.timeout(1.)
    def test4_Q_terms(self):
        """Test Q_terms"""
        # load locals
        Q_terms = self.notebook_locals['Q_terms']
        derivs = self.notebook_locals['derivs']

        np.random.seed(7)
        x = np.random.rand(5)
        u = np.random.rand(2)

        l_x, l_u, l_xx, l_ux, l_uu, f_x, f_u = derivs.stage(x, u)
        V_x, V_xx = derivs.final(x + 0.1)
        Q_x, Q_u, Q_xx, Q_ux, Q_uu = Q_terms(l_x, l_u, l_xx, l_ux, l_uu, f_x,
                                             f_u, V_x, V_xx)

        Q_x_hash = hash(tuple(np.ndarray.flatten(self.round(Q_x, 4))))
        Q_u_hash = hash(tuple(np.ndarray.flatten(self.round(Q_u, 4))))
        Q_xx_hash = hash(tuple(np.ndarray.flatten(self.round(Q_xx, 4))))
        Q_ux_hash = hash(tuple(np.ndarray.flatten(self.round(Q_ux, 4))))
        Q_uu_hash = hash(tuple(np.ndarray.flatten(self.round(Q_uu, 4))))

        self.assertEqual(Q_x_hash, 7805697548178880088, "Incorrect Q_x")
        self.assertEqual(Q_u_hash, 980374240801014371, "Incorrect Q_u")
        self.assertEqual(Q_xx_hash, 7557069004417576783, "Incorrect Q_xx")
        self.assertEqual(Q_ux_hash, -1344468598834810301, "Incorrect Q_ux")
        self.assertEqual(Q_uu_hash, -1135300939716028365, "Incorrect Q_uu")

    @weight(3)
    @timeout_decorator.timeout(1.)
    def test5_gains(self):
        """Test gains"""
        # load locals
        gains = self.notebook_locals['gains']

        np.random.seed(7)
        Q_u = np.random.randn(2)
        Q_uu = np.random.randn(2, 2)
        Q_uu = 0.5 * (Q_uu + Q_uu.T)
        Q_ux = np.random.randn(2, 5)
        k, K = gains(Q_uu, Q_u, Q_ux)

        k_hash = hash(tuple(np.ndarray.flatten(self.round(k, 4))))
        K_hash = hash(tuple(np.ndarray.flatten(self.round(K, 4))))

        self.assertEqual(k_hash, -5624267973007621395, "Incorrect k gain")

        self.assertEqual(K_hash, -4694246526328938179, "Incorrect K gain")

    @weight(3)
    @timeout_decorator.timeout(1.)
    def test6_V_terms(self):
        """Test V_terms"""
        # load locals
        V_terms = self.notebook_locals['V_terms']
        gains = self.notebook_locals['gains']

        np.random.seed(7)
        Q_x = np.random.randn(5)
        Q_u = np.random.randn(2)
        Q_uu = np.random.randn(2, 2)
        Q_uu = 0.5 * (Q_uu + Q_uu.T)
        Q_ux = np.random.randn(2, 5)
        Q_xx = np.random.randn(5, 5)
        Q_xx = 0.5 * (Q_xx + Q_xx.T)
        k, K = gains(Q_uu, Q_u, Q_ux)

        V_x, V_xx = V_terms(Q_x, Q_u, Q_xx, Q_ux, Q_uu, K, k)

        V_x_hash = hash(tuple(np.ndarray.flatten(self.round(V_x, 4))))
        V_xx_hash = hash(tuple(np.ndarray.flatten(self.round(V_xx, 4))))

        self.assertEqual(V_x_hash, 2222446538856429062, "Incorrect V_x")

        self.assertEqual(V_xx_hash, 1880932214702358488, "Incorrect V_xx")

    @weight(2)
    @timeout_decorator.timeout(1.)
    def test7_forward_pass(self):
        """Test forward_pass"""
        # load locals
        forward_pass = self.notebook_locals['forward_pass']

        np.random.seed(7)
        N = 5
        n_u = 2
        n_x = 5
        k = np.random.randn(2)
        K = np.random.randn(2, 5)
        u_trj = np.random.randn(N - 1, n_u)
        x_trj = np.random.randn(N, n_x)
        k_trj = np.random.randn(u_trj.shape[0], u_trj.shape[1])
        K_trj = np.random.randn(u_trj.shape[0], u_trj.shape[1], x_trj.shape[1])
        x_trj_new, u_trj_new = forward_pass(x_trj, u_trj, k_trj, K_trj)

        x_trj_new_hash = hash(
            tuple(np.ndarray.flatten(self.round(x_trj_new, 4))))
        u_trj_new_hash = hash(
            tuple(np.ndarray.flatten(self.round(u_trj_new, 4))))

        self.assertEqual(x_trj_new_hash, 8614906876295478816,
                         "Incorrect x_trj_new")

        self.assertEqual(u_trj_new_hash, 5430054884692290987,
                         "Incorrect u_trj_new")

    @weight(2)
    @timeout_decorator.timeout(1.)
    def test8_backward_pass(self):
        """Test backward_pass"""
        # load locals
        backward_pass = self.notebook_locals['backward_pass']

        np.random.seed(7)
        N = 5
        n_u = 2
        n_x = 5
        k = np.random.randn(2)
        K = np.random.randn(2, 5)
        u_trj = np.random.randn(N - 1, n_u)
        x_trj = np.random.randn(N, n_x)
        k_trj, K_trj, expected_cost_redu = backward_pass(x_trj, u_trj, 100.0)

        k_trj_hash = hash(tuple(np.ndarray.flatten(self.round(k_trj, 4))))
        K_trj_hash = hash(tuple(np.ndarray.flatten(self.round(K_trj, 4))))

        self.assertEqual(k_trj_hash, 4400565416540718551, "Incorrect k_trj")

        self.assertEqual(K_trj_hash, 5118135117981053517, "Incorrect K_trj")

    @staticmethod
    def round(n, decimals=0):
        multiplier = 10**decimals
        return np.round(n * multiplier) / multiplier
