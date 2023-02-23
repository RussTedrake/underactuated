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

        f_target_test_target = [
            0.36531949939751773,
            2.25991775105549,
            -0.05889795147375571,
            0.5476245815855634,
            0.8066768897676444,
        ]

        np.random.seed(7)
        n_rands = 5
        for i in range(n_rands):
            N = 5
            n_u = 2
            x0 = np.array([1, 0, 0, 1, 0])
            u_trj = np.random.randn(N - 1, n_u) * 0.4
            f_eval_test = np.random.randn(N) @ f(x0, u_trj) @ np.random.randn(N)
            msg = f'The discrete dynamics are incorrect.' + \
                'Expected output {f_target_test_target[i]} got {f_eval_test}'
            self.assertAlmostEqual(f_eval_test,
                                   f_target_test_target[i],
                                   msg=msg)

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

        Q_x_test_val = (Q_x @ np.random.randn(Q_x.shape[0]))
        Q_u_test_val = (Q_u @ np.random.randn(Q_u.shape[0]))
        Q_xx_test_val = (np.random.randn(Q_xx.shape[0]) @ Q_xx
                         @ np.random.randn(Q_xx.shape[0]))
        Q_ux_test_val = (np.random.randn(Q_ux.shape[0]) @ Q_ux
                         @ np.random.randn(Q_ux.shape[1]))
        Q_uu_test_val = (np.random.randn(Q_uu.shape[0]) @ Q_uu
                         @ np.random.randn(Q_uu.shape[0]))

        Q_x_test_target = 1.1817013977437874
        Q_u_test_target = -0.1716142906672626
        Q_xx_test_target = 0.5337352394014466
        Q_ux_test_target = 0.5156605288974515
        Q_uu_test_target = -0.28166353613679074

        self.assertAlmostEqual(Q_x_test_val,
                               Q_x_test_target,
                               msg="Incorrect Q_x")
        self.assertAlmostEqual(Q_u_test_val,
                               Q_u_test_target,
                               msg="Incorrect Q_u")
        self.assertAlmostEqual(Q_xx_test_val,
                               Q_xx_test_target,
                               msg="Incorrect Q_xx")
        self.assertAlmostEqual(Q_ux_test_val,
                               Q_ux_test_target,
                               msg="Incorrect Q_ux")
        self.assertAlmostEqual(Q_uu_test_val,
                               Q_uu_test_target,
                               msg="Incorrect Q_uu")

    @weight(3)
    @timeout_decorator.timeout(1.)
    def test5_gains(self):
        """Test gains"""
        # load locals
        gains = self.notebook_locals['gains']

        np.random.seed(8)
        Q_u = np.random.randn(2)
        Q_uu = np.random.randn(2, 2)
        Q_uu = 0.5 * (Q_uu + Q_uu.T)
        Q_ux = np.random.randn(2, 5)
        k, K = gains(Q_uu, Q_u, Q_ux)

        test_inputs = 10 * np.random.randn(k.shape[0], 5)

        k_test_val = (k @ test_inputs).sum()
        K_test_val = (test_inputs.T @ K).sum()

        self.assertAlmostEqual(k_test_val,
                               10.712481345027399,
                               msg="Incorrect k gain")

        self.assertAlmostEqual(K_test_val,
                               -12.949530490536842,
                               msg="Incorrect K gain")

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
        test_inputs = np.random.randn(V_x.shape[0])

        V_x_test_val = V_x @ test_inputs
        V_xx_test_val = test_inputs @ V_xx @ test_inputs

        self.assertAlmostEqual(V_x_test_val,
                               -2.113959925580339,
                               msg="Incorrect V_x")

        self.assertAlmostEqual(V_xx_test_val,
                               35.931091753768,
                               msg="Incorrect V_xx")

        # this tests whether the student simplified the V_terms expression
        k2, K2 = 10 * np.random.randn(*k.shape), 10 * np.random.randn(*K.shape)
        V_x2, V_xx2 = V_terms(Q_x, Q_u, Q_xx, Q_ux, Q_uu, K2, k2)

        V_x2_test_val = V_x2 @ test_inputs
        V_xx2_test_val = test_inputs @ V_xx2 @ test_inputs

        self.assertNotAlmostEqual(
            V_x2_test_val,
            2.484304411129088,
            msg="""It appears you may have simplified the expression for V_x.
            Do not do this""")
        self.assertNotAlmostEqual(
            V_xx2_test_val,
            265.0164583112695,
            msg="""It appears you may have simplified the expression for V_xx.
            Do not do this""")

        self.assertAlmostEqual(V_x2_test_val,
                               -6.043415622930539,
                               msg="Incorrect V_x")

        self.assertAlmostEqual(V_xx2_test_val,
                               -52.01732694080634,
                               msg="Incorrect V_xx")

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
        u_trj = np.random.randn(N - 1, n_u)
        x_trj = np.random.randn(N, n_x)
        k_trj = np.random.randn(u_trj.shape[0], u_trj.shape[1])
        K_trj = np.random.randn(u_trj.shape[0], u_trj.shape[1], x_trj.shape[1])
        x_trj_new, u_trj_new = forward_pass(x_trj, u_trj, k_trj, K_trj)

        x_test = np.random.randn(x_trj_new.shape[0])
        u_test = np.random.randn(u_trj_new.shape[0])

        x_trj_new_test_val = x_test @ x_trj_new @ x_test
        u_trj_new_test_val = (u_test @ u_trj_new).sum()

        self.assertAlmostEqual(x_trj_new_test_val,
                               -1.0615491271322297,
                               msg="Incorrect x_trj_new")

        self.assertAlmostEqual(u_trj_new_test_val,
                               -10.649763284585609,
                               msg="Incorrect u_trj_new")

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

        k_trj, K_trj, _ = backward_pass(x_trj, u_trj, 100.0)

        r_test = 10 * np.random.randn(k_trj.shape[1])
        l_test = 10 * np.random.randn(k_trj.shape[0])

        k_trj_test_val = l_test @ k_trj @ r_test
        K_trj_test_val = np.einsum("i, ijk, j -> k", l_test, K_trj,
                                   r_test).sum()

        self.assertAlmostEqual(k_trj_test_val,
                               -5.380572857461254,
                               msg="Incorrect k_trj")

        self.assertAlmostEqual(K_trj_test_val,
                               3.770894929116537,
                               msg="Incorrect K_trj")
