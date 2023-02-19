import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestCompassGaitLimitCycle(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    # starts by testing the constraints we already implemented
    # but gives zero points for each test

    @weight(0)
    @timeout_decorator.timeout(3.)
    def test_time_intervals(self):
        """Time intervals are within bounds"""
        # retrieve variables from notebook
        T = self.notebook_locals['T']
        h_opt = self.notebook_locals['h_opt']
        h_min = self.notebook_locals['h_min']
        h_max = self.notebook_locals['h_max']

        # check one per time
        tol = 1e-4
        for t in range(T):
            self.assertTrue(h_opt[t] >= h_min - tol,
                            msg=f'Time interval lower bound violated at time '
                            + f'step {t}.')
            self.assertTrue(h_opt[t] <= h_max + tol,
                            msg=f'Time interval upper bound violated at time '
                            + f'step {t}.')

    @weight(0)
    @timeout_decorator.timeout(3.)
    def test_dynamic_feasibility(self):
        """Implicit Euler equations are verified"""
        # retrieve variables from notebook
        T = self.notebook_locals['T']
        h = self.notebook_locals['h_opt']
        q = self.notebook_locals['q_opt']
        qd = self.notebook_locals['qd_opt']
        qdd = self.notebook_locals['qdd_opt']
        f = self.notebook_locals['f_opt']
        manipulator_equations = self.notebook_locals['manipulator_equations']

        # link configurations, velocities, accelerations using implicit Euler
        for t in range(T):
            np.testing.assert_allclose(
                q[t + 1],
                q[t] + h[t] * qd[t + 1],
                atol=1e-5,
                rtol=1e-5,
                err_msg='Configurations q and velocities qd do not verify '
                + f'the Implicit Euler rule at time step {t}.')
            np.testing.assert_allclose(
                qd[t + 1],
                qd[t] + h[t] * qdd[t],
                atol=1e-5,
                rtol=1e-5,
                err_msg='Velocities qd and accelerations qdd do not verify '
                + f'the Implicit Euler rule at time step {t}.')
            vars = np.concatenate((q[t + 1], qd[t + 1], qdd[t], f[t]))
            manipulator_equations_violation = manipulator_equations(vars)
            np.testing.assert_allclose(
                manipulator_equations_violation,
                np.zeros(q.shape[1]),
                atol=1e-5,
                rtol=1e-5,
                err_msg='Manipulator equations are not verified at time '
                + f'step {t}.')

    @weight(0)
    @timeout_decorator.timeout(3.)
    def test_impulsive_collision(self):
        """Impulsive collision is verified at the heel strike"""
        # retrieve variables from notebook
        q = self.notebook_locals['q_opt']
        qd = self.notebook_locals['qd_opt']
        qd_post = self.notebook_locals['qd_post_opt']
        imp = self.notebook_locals['imp_opt']
        reset_velocity_heelstrike = self.notebook_locals[
            'reset_velocity_heelstrike']

        # link pre and post impact velocities
        vars = np.concatenate((q[-1], qd[-1], qd_post, imp))
        reset_velocity_heelstrike_violation = reset_velocity_heelstrike(vars)
        np.testing.assert_allclose(
            reset_velocity_heelstrike_violation,
            np.zeros(q.shape[1] + imp.size),
            atol=1e-5,
            rtol=1e-5,
            err_msg='Impulsive collision equations are not verified.')

    @weight(0)
    @timeout_decorator.timeout(3.)
    def test_periodicity(self):
        """Periodicity of the walking gait"""
        # retrieve variables from notebook
        q = self.notebook_locals['q_opt']
        qd = self.notebook_locals['qd_opt']
        qd_post = self.notebook_locals['qd_post_opt']

        # periodicity of the configuration
        np.testing.assert_allclose(
            q[0],
            -q[-1],
            atol=1e-5,
            rtol=1e-5,
            err_msg='Configuration vector q does not verify periodicity '
            + 'conditions.')

        # periodicity of the  velocities
        np.testing.assert_allclose(
            qd[0],
            np.array([0, 0, qd_post[2] + qd_post[3], -qd_post[3]]),
            atol=1e-5,
            rtol=1e-5,
            err_msg='Velocity vector qd does not verify periodicity '
            + 'conditions.')

    # tests constraints implemented by the students
    # does not use the same expressions that must be used in the constraint
    # e.g. checks that a linear constraint is verified by evaluating a nonlinear
    # function

    @weight(3)
    @timeout_decorator.timeout(3.)
    def test_stance_foot_on_ground(self):
        """Stance foot on the ground for all times"""
        # retrieve variables from notebook
        T = self.notebook_locals['T']
        q = self.notebook_locals['q_opt']
        for t in range(T + 1):
            self.assertAlmostEqual(
                np.linalg.norm(q[t, :2]),
                0,
                msg=f'Stance foot is not on the ground at time step {t}.')

    @weight(2)
    @timeout_decorator.timeout(3.)
    def test_swing_foot_on_ground(self):
        """Swing foot on the ground at time zero"""
        # retrieve variables from notebook
        q = self.notebook_locals['q_opt']
        swing_foot_height = self.notebook_locals['swing_foot_height']
        self.assertAlmostEqual(
            swing_foot_height(q[0])[0],
            0,
            msg='Swing foot is not on the ground at time zero.')

    @weight(3)
    @timeout_decorator.timeout(3.)
    def test_no_penetration(self):
        """No penetration of the swing foot in the ground for all times"""
        # retrieve variables from notebook
        T = self.notebook_locals['T']
        q = self.notebook_locals['q_opt']
        swing_foot_height = self.notebook_locals['swing_foot_height']
        tol = 1e-4
        for t in range(T + 1):
            self.assertGreater(
                swing_foot_height(q[t])[0],
                -tol,
                msg=f'Swing foot in penetration with ground at time step {t}.')

    @weight(3)
    @timeout_decorator.timeout(3.)
    def test_stance_foot_friction(self):
        """Stance-foot contact force in friction cone for all times"""
        # retrieve variables from notebook
        T = self.notebook_locals['T']
        friction = self.notebook_locals['friction']
        f = self.notebook_locals['f_opt']
        tol = 1e-4
        for t in range(T):
            self.assertGreater(
                f[t, 1],
                -tol,
                msg='Stance-foot normal contact force is negative at time '
                + f'step {t}.')
            self.assertLess(
                np.abs(f[t, 0]),
                friction * f[t, 1] + tol,
                msg='Stance-foot contact force outside friction cone at time '
                + f'step {t}.')

    @weight(2)
    @timeout_decorator.timeout(3.)
    def test_swing_foot_friction(self):
        """Swing-foot impulse in friction cone"""
        # retrieve variables from notebook
        friction = self.notebook_locals['friction']
        imp = self.notebook_locals['imp_opt']
        tol = 1e-4
        self.assertGreater(
            imp[1],
            -tol,
            msg='Normal heel-strike impulse is negative at time step {t}.')
        self.assertLess(np.abs(imp[0]),
                        friction * imp[1] + tol,
                        msg='Heel-strike impulse outside friction cone')
