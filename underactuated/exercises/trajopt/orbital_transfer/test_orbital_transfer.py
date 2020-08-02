import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestOrbitalTransfer(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    def trajopt_feasibility(self):
        """Test feasibility of the trajopt solution"""
        # load locals
        universe = self.notebook_locals['universe']
        thrust = self.notebook_locals['thrust_opt']
        state = self.notebook_locals['state_opt']
        time_steps = self.notebook_locals['time_steps']
        time_interval = self.notebook_locals['time_interval']

        # initial state in Earth orbit
        residuals = universe.constraint_state_to_orbit(state[0], 'Earth')
        np.testing.assert_array_almost_equal(
            residuals,
            np.zeros(residuals.shape),
            err_msg='The initial state in not on the Earth orbit.')

        # final state in Mars orbit
        residuals = universe.constraint_state_to_orbit(state[-1], 'Mars')
        np.testing.assert_array_almost_equal(
            residuals,
            np.zeros(residuals.shape),
            # decimal=4,
            err_msg='The final state in not on the Mars orbit.')

        # discretized dynamics
        for t in range(time_steps):
            residuals = universe.rocket_discrete_dynamics(
                state[t], state[t + 1], thrust[t], time_interval)
            np.testing.assert_array_almost_equal(
                residuals,
                np.zeros(residuals.shape),
                err_msg=f'Discrete dynamics at time step {t} is not verified.')

    @weight(3)
    @timeout_decorator.timeout(1.)
    def test_thrust_limits(self):
        """Test thrust limits"""
        # check feasibility first
        self.trajopt_feasibility()

        # load locals
        universe = self.notebook_locals['universe']
        thrust = self.notebook_locals['thrust_opt']
        time_steps = self.notebook_locals['time_steps']

        # check thrust limits
        tol = 1e-3
        thrust_limit = universe.rocket.thrust_limit
        for t in range(time_steps):
            self.assertTrue(np.linalg.norm(thrust[t]) <= thrust_limit + tol,
                            msg=f'Thrust limit violated at time step {t}.')

    @weight(3)
    @timeout_decorator.timeout(1.)
    def test_velocity_limits(self):
        """Test velocity limits"""
        # check feasibility first
        self.trajopt_feasibility()

        # load locals
        universe = self.notebook_locals['universe']
        velocity = self.notebook_locals['state_opt'][:, 2:]
        time_steps = self.notebook_locals['time_steps']

        # check velocity limits
        tol = 1e-3
        velocity_limit = universe.rocket.velocity_limit
        for t in range(time_steps + 1):
            self.assertTrue(np.linalg.norm(velocity[t]) <= velocity_limit + tol,
                            msg=f'Velocity limit violated at time step {t}.')

    @weight(6)
    @timeout_decorator.timeout(1.)
    def test_asteroid_collision_limits(self):
        """Test asteroid collision"""
        # check feasibility first
        self.trajopt_feasibility()

        # load locals
        universe = self.notebook_locals['universe']
        state = self.notebook_locals['state_opt']
        time_steps = self.notebook_locals['time_steps']
        n_asteroids = self.notebook_locals['n_asteroids']

        # avoid collision with asteroids
        tol = 1e-3
        for i in range(n_asteroids):
            asteroid_orbit = universe.get_planet(f'Asteroid_{i}').orbit
            for t in range(time_steps + 1):
                p = universe.position_wrt_planet(state[t], f'Asteroid_{i}')
                self.assertTrue(
                    p.dot(p) >= asteroid_orbit**2 - tol,
                    msg=f'Collision with asteroid {i} at time step {t}.')

    @weight(3)
    @timeout_decorator.timeout(1.)
    def test_fuel_consumption_limits(self):
        """Test fuel consumption"""
        # check feasibility first
        self.trajopt_feasibility()

        # load locals
        universe = self.notebook_locals['universe']
        thrust = self.notebook_locals['thrust_opt']
        time_interval = self.notebook_locals['time_interval']

        # check maximum consumption
        consumption = time_interval * sum(t.dot(t) for t in thrust)
        self.assertTrue(
            consumption <= 250,
            msg=f'Fuel consumption is {consumption}, greater than 250.')
