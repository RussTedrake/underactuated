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

        random_matrix_test_vect = np.array(
            [[
                1.6905e+00, -4.6590e-01, 3.2800e-02, 4.0750e-01, -7.8890e-01,
                2.1000e-03, -9.0000e-04, -1.7547e+00, 1.0177e+00, 6.0050e-01
            ],
             [
                 -6.2540e-01, -1.7150e-01, 5.0530e-01, -2.6140e-01, -2.4270e-01,
                 -1.4532e+00, 5.5460e-01, 1.2390e-01, 2.7450e-01, -1.5265e+00
             ],
             [
                 1.6507e+00, 1.5430e-01, -3.8710e-01, 2.0291e+00, -4.5400e-02,
                 -1.4507e+00, -4.0520e-01, -2.2883e+00, 1.0494e+00, -4.1650e-01
             ],
             [
                 -7.4260e-01, 1.0725e+00, -1.6511e+00, 5.3540e-01, -2.0644e+00,
                 -6.6220e-01, -1.2042e+00, 1.4620e+00, 1.7662e+00, -3.2940e-01
             ],
             [
                 8.4070e-01, -1.8000e-01, 5.6810e-01, -7.5280e-01, -1.7083e+00,
                 -1.8031e+00, 3.8310e-01, 2.2476e+00, 2.6940e-01, -5.2460e-01
             ]])
        u_trj_all = np.array([[[0.76480754, 0.09492074],
                               [0.04057359, 0.10103109],
                               [-0.05295088, -0.12379054],
                               [-0.57398539, 0.20064965]],
                              [[-0.03791018, 0.47723437],
                               [-0.14752739, -0.76254795],
                               [-0.03984425, 0.67981492],
                               [-0.15336925, -0.35594274]],
                              [[-0.47743677, -0.42000672],
                               [-0.12007749, -0.47199284],
                               [0.59905565, -0.11305409],
                               [0.04345935, 0.57529581]],
                              [[0.60132741, -0.08509319],
                               [0.13278969, 0.29401063],
                               [-0.07714218, -0.71120514],
                               [0.26188228, 0.35774092]],
                              [[0.16620105, -0.36941786],
                               [-0.07841092, -0.23630793],
                               [-0.11988449, 0.51875408],
                               [0.61183185, 0.26776728]]])
        np.random.seed(7)
        n_rands = 5
        randomized_matrix_test_evals = np.zeros(
            (n_rands, random_matrix_test_vect.shape[0],
             random_matrix_test_vect.shape[1]))

        for i in range(n_rands):
            N = 5
            n_u = 2
            x0 = np.array([1, 0, 0, 1, 0])
            u_trj = u_trj_all[i, :, :]
            randomized_matrix_test_evals[i, :, :] = f(
                x0, u_trj) @ random_matrix_test_vect

        true_test_values = np.array([
            0.5027427513713181, 1.1127426840962125, 0.2711, 3.14382111661095,
            1.6792297046356366, -1.2936517104174001, 0.5156284227267528,
            1.026328389485229, 0.2711, 2.9516087593956315, 1.6570611581160513,
            -1.1823420074097, 0.5377511938262911, 1.111426980030478, 0.2711,
            2.8488181016658225, 1.5303435600857451, -1.1637875215998,
            0.5021827947119019, 1.1463243983464397, 0.2711, 3.127901019692183,
            1.5707361888389506, -1.2808617668211002, 0.4906033329313717,
            1.051074922504677, 0.2711, 2.9843757725810467, 1.5831104990613687,
            -1.2393563286576001
        ])

        test_inds = [(4, 9), (3, 3), (0, 9), (2, 8), (4, 0), (1, 6)]

        idx = 0
        for i in range(n_rands):
            for ind in test_inds:
                self.assertTrue(
                    abs(randomized_matrix_test_evals[i, ind[0], ind[1]]
                        - true_test_values[idx]) < 1e-10)
                idx += 1

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

        x_test_vects = np.array(
            [[
                0.35358551, 0.25400233, 1.04183824, 0.60648842, -0.37488226,
                0.63030166, 1.43516303, -0.11720509, -0.73816781, 0.45447108
            ],
             [
                 -0.54842885, 0.2021185, 1.11272554, -0.50839559, 1.91690636,
                 -1.38785085, 0.79585093, 0.56527635, -1.10668652, -2.39396727
             ],
             [
                 0.06250172, -0.67038165, 1.15449539, 0.37880842, 0.69580636,
                 -1.17849924, -0.34865563, -0.46440773, -1.21665464, 0.27560803
             ],
             [
                 0.48143516, 0.38062968, 0.03622711, -0.51749844, -0.77796907,
                 -0.53073556, -0.26416401, -1.36652253, 0.75655156, 0.48027434
             ],
             [
                 0.48762617, -0.98053378, 0.21910346, 1.3567134, 0.86464873,
                 -0.61355763, -0.56614591, -0.17953522, 1.9086429, 0.23986947
             ]])
        u_test_vects = np.array([[
            -1.06344985, -0.21228429, 0.92569535, -1.7279359, 1.53067224,
            -0.23343633, 0.29719528, 0.99286434, -0.93597859, -0.95960851
        ],
                                 [
                                     -0.19904275, -0.61285757, -1.25337305,
                                     -0.3222009, 2.71396693, 1.62985742,
                                     1.13361691, -0.87478952, -0.39614971,
                                     -0.43777013
                                 ]])

        Q_x_hash = (Q_x @ x_test_vects).sum()
        Q_u_hash = (Q_u @ u_test_vects).sum()
        Q_xx_hash = (x_test_vects.T @ Q_xx @ x_test_vects).sum()
        Q_ux_hash = (u_test_vects.T @ Q_ux @ x_test_vects).sum()
        Q_uu_hash = (u_test_vects.T @ Q_uu @ u_test_vects).sum()

        tol = 1e-8
        self.assertLessEqual(abs(Q_x_hash, 10.649017611427563), tol,
                             "Incorrect Q_x")
        self.assertLessEqual(Q_u_hash, 0.3153336670736354, tol, "Incorrect Q_u")
        self.assertLessEqual(Q_xx_hash, -64.8723176741333, tol,
                             "Incorrect Q_xx")
        self.assertLessEqual(Q_ux_hash, 0.36646551886176343, tol,
                             "Incorrect Q_ux")
        self.assertLessEqual(Q_uu_hash, 0.8043560396818086, tol,
                             "Incorrect Q_uu")

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

        self.assertTrue(np.allclose(Q_uu @ k @ Q_uu @ K, Q_u @ Q_ux))

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

        test_vects = np.array(
            [[
                0.38312185, 2.24759505, 0.26941163, -0.52460462, 1.91201886,
                0.23730185, 0.10143399, 0.25257774, -0.1323772, -0.30947634
            ],
             [
                 -1.43496347, 0.50162412, -0.09477545, 1.19308592, -0.36881847,
                 -1.90636988, -0.09961063, 1.6995373, -0.38342312, -0.88985686
             ],
             [
                 -1.19359192, -1.05001681, -0.30019374, -1.17998209, 1.49763912,
                 -0.28263524, 0.10864837, 1.43823952, 1.50331852, -0.21273297
             ],
             [
                 0.33197422, 0.73502658, -0.19285546, -1.77801285, 0.6547057,
                 0.8943523, 0.41550261, -0.92354466, -0.19602731, -0.59076982
             ],
             [
                 -0.29971124, 1.29688519, 1.52957963, 0.66941819, 0.54874512,
                 0.67662899, -0.01224219, -0.07566346, -0.67364519, -0.05586745
             ]])

        self.assertLessEqual(
            np.abs((V_x @ test_vects).sum() - 5.228032247714198), 1e-8,
            "Incorrect V_x")

        self.assertLessEqual(
            np.abs((V_xx @ test_vects).sum() - 5.351734731679263), 1e-8,
            "Incorrect V_xx")

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
        k = np.array([1.6905257, -0.46593737])
        K = np.array([[
            3.28201637e-02, 4.07516283e-01, -7.88923029e-01, 2.06557291e-03,
            -8.90385858e-04
        ],
                      [
                          -1.75472431, 1.01765801, 6.00498516e-01,
                          -6.25428974e-01, -1.71548261e-01
                      ]])
        u_trj = np.array([[0.50529937, -0.26135642], [-0.24274908, -1.45324141],
                          [0.55458031, 0.12388091], [0.27445992, -1.52652453]])
        x_trj = np.array(
            [[1.65069969, 0.15433554, -0.38713994, 2.02907222, -0.04538603],
             [-1.4506787, -0.40522786, -2.2883151, 1.04939655, -0.41647432],
             [-0.74255353, 1.07247013, -1.65107559, 0.53542936, -2.0644148],
             [-0.66215934, -1.20421985, 1.46197563, 1.76616088, -0.32941375],
             [0.84073324, -0.1799864, 0.56806189, -0.7528372, -1.7083392]])
        k_trj = np.array([[-1.80309866, 0.38312185], [2.24759505, 0.26941163],
                          [-0.52460462, 1.91201886], [0.23730185, 0.10143399]])
        K_trj = np.array(
            [[[0.25257774, -0.1323772, -0.30947634, -1.43496347, 0.50162412],
              [-0.09477545, 1.19308592, -0.36881847, -1.90636988, -0.09961063]],
             [[1.6995373, -0.38342312, -0.88985686, -1.19359192, -1.05001681],
              [-0.30019374, -1.17998209, 1.49763912, -0.28263524, 0.10864837]],
             [[1.43823952, 1.50331852, -0.21273297, 0.33197422, 0.73502658],
              [-0.19285546, -1.77801285, 0.6547057, 0.8943523, 0.41550261]],
             [[-0.92354466, -0.19602731, -0.59076982, -0.29971124, 1.29688519],
              [1.52957963, 0.66941819, 0.54874512, 0.67662899, -0.01224219]]])
        x_trj_new, u_trj_new = forward_pass(x_trj, u_trj, k_trj, K_trj)

        x_trj_new_hash = x_trj_new.sum()
        u_trj_new_hash = u_trj_new.sum()

        tol = 1e-8

        self.assertLessEqual(np.abs(x_trj_new_hash - 21.53616248167206), tol,
                             "Incorrect x_trj_new")

        self.assertLessEqual(np.abs(u_trj_new_hash - 16.936959128086087), tol,
                             "Incorrect u_trj_new")

    @weight(2)
    @timeout_decorator.timeout(1.)
    def test8_backward_pass(self):
        """Test backward_pass"""
        # load locals
        backward_pass = self.notebook_locals['backward_pass']

        np.random.seed(7)
        u_trj = np.array([[0.50529937, -0.26135642], [-0.24274908, -1.45324141],
                          [0.55458031, 0.12388091], [0.27445992, -1.52652453]])
        x_trj = np.array(
            [[1.65069969, 0.15433554, -0.38713994, 2.02907222, -0.04538603],
             [-1.4506787, -0.40522786, -2.2883151, 1.04939655, -0.41647432],
             [-0.74255353, 1.07247013, -1.65107559, 0.53542936, -2.0644148],
             [-0.66215934, -1.20421985, 1.46197563, 1.76616088, -0.32941375],
             [0.84073324, -0.1799864, 0.56806189, -0.7528372, -1.7083392]])
        k_trj, K_trj, expected_cost_redu = backward_pass(x_trj, u_trj, 100.0)

        k_trj_hash = k_trj.sum()
        K_trj_hash = K_trj_hash.sum()
        tol = 1e-8
        self.assertLessEqual(np.abs(k_trj_hash - 0.03498055919245241), tol,
                             "Incorrect k_trj")

        self.assertLessEqual(np.abs(K_trj_hash + 0.019062339335679766), tol,
                             "Incorrect K_trj")

    @staticmethod
    def round(n, decimals=0):
        multiplier = 10**decimals
        return np.round(n * multiplier) / multiplier
