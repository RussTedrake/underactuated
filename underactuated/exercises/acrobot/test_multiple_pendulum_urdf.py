import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np

class TestMultiplePendulumURDF(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(1)
    @timeout_decorator.timeout(1.)
    def test_x_star_single_state(self):
        """Test x_star for single pendulum"""
        # note: all prints here go to the output item in the json file
        x_star_obtained = self.notebook_locals['x_star_single_pendulum']
        assert (len(x_star_obtained) == 4), "x_star for single pendulum must have dimension 4"
        x_star_target = np.array([0, np.pi, 0, 0])
        np.testing.assert_array_almost_equal(x_star_obtained, x_star_target, 
                    err_msg='unstable equilibrium for single pendulum does not match')

    @weight(1)
    @timeout_decorator.timeout(1.)
    def test_x_star_double_state(self):
        """Test x_star for double pendulum"""
        # note: all prints here go to the output item in the json file
        x_star_obtained = self.notebook_locals['x_star_double_pendulum']
        assert (len(x_star_obtained) == 6), "x_star for double pendulum must have dimension 6"
        x_star_target = np.array([0, np.pi, np.pi, 0, 0, 0])
        np.testing.assert_array_almost_equal(x_star_obtained, x_star_target, 
                    err_msg='unstable equilibrium for double pendulum does not match')

    @weight(3)
    @timeout_decorator.timeout(10.)
    def test_single_pendulum_urdf(self):
        """Test single-pendulum cart-pole dynamics"""
        # note: all prints here go to the output item in the json file
        obtained_single_pend_urdf = self.notebook_locals['single_pendulum_urdf']
        cartpole_fn = self.notebook_locals['cartpole_balancing']
        x_star = np.array([0, np.pi, 0, 0])
        Q = np.diag((10., 10., 1., 1.))
        R = np.eye(1)
        state_lst = np.array([[0.05, np.pi*0.95, -0.01, 0.01*np.pi],
                              [-0.01, np.pi*1.1,   0.1, 0.        ],
                              [0.2,  np.pi*0.5,   0.01, 0.        ],
                              [0.0,  np.pi,       0.0,  0.        ],
                              [-2,   np.pi*1.2,   0.,  -0.1*np.pi ]])
        diagram = cartpole_fn(obtained_single_pend_urdf, x_star, Q, R)
        cartpole = diagram.GetSubsystemByName("cartpole")
        context = cartpole.CreateDefaultContext()
        cartpole.get_actuation_input_port().FixValue(context, [0])
        result_lst = []
        for state in state_lst:
            context.SetContinuousState(state)
            result_lst.append(cartpole.EvalTimeDerivatives(context).CopyToVector())
        result = np.array(result_lst)
        target_derivatives = np.array([[-1.00000000e-02,  3.14159265e-02, -1.47937117e+00,
                                        -2.99577976e+00],
                                        [ 1.00000000e-01,  0.00000000e+00,  2.63177456e+00,
                                        5.53442305e+00],
                                        [ 1.00000000e-02,  0.00000000e+00,  3.00344627e-16,
                                        -9.81000000e+00],
                                        [ 0.00000000e+00,  0.00000000e+00, -1.20137851e-15,
                                        -2.40275702e-15],
                                        [ 0.00000000e+00, -3.14159265e-01,  3.42396821e+00,
                                        8.53622180e+00]])
        np.testing.assert_array_almost_equal(result, target_derivatives, decimal=6,
                    err_msg='the dynamics for the single pendulum do not match')

    @weight(3)
    @timeout_decorator.timeout(10.)
    def test_double_pendulum_urdf(self):
        """Test double-pendulum cart-pole dynamics"""
        # note: all prints here go to the output item in the json file
        obtained_double_pend_urdf = self.notebook_locals['double_pendulum_urdf']
        cartpole_fn = self.notebook_locals['cartpole_balancing']
        x_star = np.array([0, np.pi, np.pi, 0, 0, 0])
        Q = np.diag((10., 10., 10., 1., 1., 1.))
        R = np.eye(1)
        state_lst = np.array([[0.05,  np.pi*0.95,  np.pi/2,  -0.01, 0.01*np.pi, 0.0       ],
                              [-0.01, np.pi*1.1,  -np.pi*0.1, 0.1,  0. ,       -0.2*np.pi ],
                              [0.2,   np.pi*0.5,   np.pi/8,   0.01, 0.0,        0.        ],
                              [0.0,   np.pi,       np.pi,     0.0,  0.0,        0.        ],
                              [-2,    np.pi*1.2,   0.01,      0.2,  0.,        -0.1*np.pi ]])
        diagram = cartpole_fn(obtained_double_pend_urdf, x_star, Q, R)
        cartpole = diagram.GetSubsystemByName("cartpole")
        context = cartpole.CreateDefaultContext()
        cartpole.get_actuation_input_port().FixValue(context, [0])
        result_lst = []
        for state in state_lst:
            context.SetContinuousState(state)
            result_lst.append(cartpole.EvalTimeDerivatives(context).CopyToVector())
        result = np.array(result_lst)
        target_derivatives = np.array([[-1.00000000e-02,  3.14159265e-02,  0.00000000e+00,
                                            -1.47937117e+00, -2.99528628e+00,  1.24520973e+01],
                                        [ 1.00000000e-01,  0.00000000e+00, -6.28318531e-01,
                                            4.39196356e+00,  9.23596755e+00, -1.36279311e+01],
                                        [ 1.00000000e-02,  0.00000000e+00,  0.00000000e+00,
                                            1.61809378e-16, -9.81000000e+00,  9.81000000e+00],
                                        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                            -2.40275702e-15, -2.40275702e-15,  4.80551404e-15],
                                        [ 2.00000000e-01,  0.00000000e+00, -3.14159265e-01,
                                            5.48279134e+00,  1.01557019e+01, -1.00624246e+01]])
        np.testing.assert_array_almost_equal(result, target_derivatives, decimal=6,
                    err_msg='the dynamics for the double pendulum do not match')

    @weight(2)
    @timeout_decorator.timeout(10.)
    def test_single_pendulum_lqr(self):
        """Test single-pendulum cart-pole LQR states"""
        # note: all prints here go to the output item in the json file
        obtained_single_pend_urdf = self.notebook_locals['single_pendulum_urdf']
        cartpole_fn = self.notebook_locals['cartpole_balancing']
        x_star = np.array([0, np.pi, 0, 0])
        Q = np.diag((10., 10., 1., 1.))
        R = np.eye(1)
        state_lst = np.array([[0.05, np.pi*0.95, -0.01, 0.01*np.pi],
                              [-0.01, np.pi*1.1,   0.1, 0.        ],
                              [0.2,  np.pi*0.5,   0.01, 0.        ],
                              [0.0,  np.pi,       0.0,  0.        ],
                              [-2,   np.pi*1.2,   0.,  -0.1*np.pi ]])
        diagram = cartpole_fn(obtained_single_pend_urdf, x_star, Q, R)
        result_lst = []
        context = diagram.CreateDefaultContext()
        for state in state_lst:
            context.SetContinuousState(state)
            result_lst.append(diagram.EvalTimeDerivatives(context).CopyToVector())
        result = np.array(result_lst)
        target_derivatives = np.array([[-1.00000000e-02,  3.14159265e-02,  7.54162904e+00, 5.91415697e+00],
                                       [ 1.00000000e-01,  0.00000000e+00, -1.45222333e+01, -1.07800079e+01],
                                       [ 1.00000000e-02,  0.00000000e+00,  4.85651106e+01, -9.81000000e+00],
                                       [ 0.00000000e+00,  0.00000000e+00, -1.20137851e-15, -2.40275702e-15],
                                       [ 0.00000000e+00, -3.14159265e-01, -2.61730991e+01, -1.54083086e+01]])
        np.testing.assert_array_almost_equal(result, target_derivatives, decimal=4,
                    err_msg='the dynamics for the single pendulum LQR do not match')

    @weight(2)
    @timeout_decorator.timeout(10.)
    def test_double_pendulum_lqr(self):
        """Test double-pendulum cart-pole LQR states"""
        # note: all prints here go to the output item in the json file
        obtained_double_pend_urdf = self.notebook_locals['double_pendulum_urdf']
        diagram_fn = self.notebook_locals['cartpole_balancing']
        x_star = np.array([0, np.pi, np.pi, 0, 0, 0])
        Q = np.diag((10., 10., 10., 1., 1., 1.))
        R = np.eye(1)
        diagram = diagram_fn(obtained_double_pend_urdf, x_star, Q, R)
        context = diagram.CreateDefaultContext()
        state_lst = np.array([[0.05,  np.pi*0.95,  np.pi/2,  -0.01, 0.01*np.pi, 0.0       ],
                              [-0.01, np.pi*1.1,  -np.pi*0.1, 0.1,  0. ,       -0.2*np.pi ],
                              [0.2,   np.pi*0.5,   np.pi/8,   0.01, 0.0,        0.        ],
                              [0.0,   np.pi,       np.pi,     0.0,  0.0,        0.        ],
                              [-2,    np.pi*1.2,   0.01,      0.2,  0.,        -0.1*np.pi ]])
        result_lst = []
        context = diagram.CreateDefaultContext()
        for state in state_lst:
            context.SetContinuousState(state)
            result_lst.append(diagram.EvalTimeDerivatives(context).CopyToVector())
        result = np.array(result_lst)
        target_derivatives = np.array([[-1.00000000e-02,  3.14159265e-02,  0.00000000e+00,
                                            -2.32240313e+01, -2.44722336e+01,  3.05274303e+01],
                                       [ 1.00000000e-01,  0.00000000e+00, -6.28318531e-01,
                                            -8.22204977e+01, -6.59570809e+01,  4.64654948e+01],
                                       [ 1.00000000e-02,  0.00000000e+00,  0.00000000e+00,
                                            2.32232750e+01, -1.69718404e+01,  3.24756807e+01],
                                       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                            -2.40275702e-15, -2.40275702e-15,  4.80551404e-15],
                                       [ 2.00000000e-01,  0.00000000e+00, -3.14159265e-01,
                                            -6.76710554e+01, -4.94569192e+01,  5.04100712e+01]])
        np.testing.assert_array_almost_equal(result, target_derivatives, decimal=4,
                    err_msg='the dynamics for the double pendulum LQR do not match')