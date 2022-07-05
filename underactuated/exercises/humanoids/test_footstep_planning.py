import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestFootstepPlanning(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        """
        Since the planner raises an error if the problem is infeasible, it is
        better to solve the optimizations inside each method rather than here in
        the __init__ method. This way if the plan is infeasible, students get
        zero points instead of an error from the test file. Note also that each
        time a test (method of this class) fails a new class is instantiated,
        hence we must re-solve the plans for each test.
        """
        super().__init__(test_name)

        # get locals
        self.notebook_locals = notebook_locals
        Terrain = self.notebook_locals['Terrain']
        self.footstep_planner = self.notebook_locals['footstep_planner']

        # empty solution of the benchmark problems
        self.terrains = {'A': Terrain([1, 1, 1, 1]), 'B': Terrain([1, 1, 1, 0])}
        self.n_steps = 8
        self.step_span = .8
        self.plans = {'A': {}, 'B': {}}

    def _solve_benchmarks(self):
        """Solves the benchmark problems."""
        for plan in ['A', 'B']:
            self.plans[plan]['vars'], self.plans[plan][
                'cost'] = self.footstep_planner(self.terrains[plan],
                                                self.n_steps, self.step_span)

    @weight(4)
    @timeout_decorator.timeout(30.)
    def test_relative_position_limits(self):
        """Tests the constraints on the relative position of the feet"""
        self._solve_benchmarks()

        # center to corner vector for the square with side step_span
        # (plus tolerance for the checks)
        tol = 1e-2
        c2c = np.ones(2) * self.step_span / 2 + tol

        # check constraint for both terrains
        for plan in ['A', 'B']:
            position_left, position_right = self.plans[plan]['vars'][:2]
            relative_position = position_left - position_right
            for t in range(self.n_steps + 1):
                msg = f'Error at foot step {t} when solving the problem ' + \
                    f'for terrain_{plan}: one foot is not contained in a ' + \
                    f'square of side {self.step_span} centered at the ' + \
                    'other foot.'
                np.testing.assert_array_less(relative_position[t], c2c, msg)
                np.testing.assert_array_less(-relative_position[t], c2c, msg)

    @weight(3)
    @timeout_decorator.timeout(30.)
    def test_one_stone_per_foot(self):
        """Tests that a foot is assigned to a single stepping stone"""
        self._solve_benchmarks()

        # check constraint for both terrains
        for plan in ['A', 'B']:
            stone_left, stone_right = self.plans[plan]['vars'][2:4]
            for t in range(self.n_steps + 1):
                msg = f'Error at foot step {t} when solving the problem ' + \
                    f'for terrain_{plan}: '
                self.assertAlmostEqual(
                    sum(stone_left[t]),
                    1,
                    places=4,
                    msg=msg + f'stone_left[{t}] do not sum up to one.')
                self.assertAlmostEqual(
                    sum(stone_right[t]),
                    1,
                    places=4,
                    msg=msg + f'stone_right[{t}] do not sum up to one.')

    @weight(5)
    @timeout_decorator.timeout(30.)
    def test_foot_in_stepping_stone(self):
        """Tests that binaries imply correctly the positioning of the feet"""
        self._solve_benchmarks()
        tol = 1e-2

        # check constraint for both terrains
        for plan in ['A', 'B']:
            position_left, position_right, \
                stone_left, stone_right = self.plans[plan]['vars'][:4]
            for t in range(self.n_steps + 1):

                # left foot
                i = np.where(np.isclose(stone_left[t], 1, atol=tol))[0][0]
                stone = self.terrains[plan].stepping_stones[i]
                np.testing.assert_array_less(
                    stone.A.dot(position_left[t]), stone.b + tol,
                    f'Solving the problem for terrain_{plan}, the binary '
                    + f'variable stone_left[{t}, {i}] is one but the left foot '
                    + f'is not in stone {i} at time {t}.')

                # right foot
                i = np.where(np.isclose(stone_right[t], 1, atol=tol))[0][0]
                stone = self.terrains[plan].stepping_stones[i]
                np.testing.assert_array_less(
                    stone.A.dot(position_right[t]), stone.b + tol,
                    f'Solving the problem for terrain_{plan}, the binary '
                    + f'variable stone_right[{t}, {i}] is one but the right '
                    + f'foot is not in stone {i} at time {t}.')

    @weight(4)
    @timeout_decorator.timeout(30.)
    def test_minimize_step_length(self):
        """
        Tests that the positioning of the feet minimized the sum of the
        square of the two norm of the step lengths.
        """
        self._solve_benchmarks()
        tol = 1e-2

        # check constraint for both terrains
        target_cost = {'A': 4.1592510194, 'B': 5.6478552756}
        for plan in ['A', 'B']:

            # same convergence check as in
            # https://github.com/RobotLocomotion/drake/blob/88c93118df507777eb3f99628d1aa7c808f81f49/solvers/branch_and_bound.cc#L711
            # with double the tolerances from
            # https://github.com/RobotLocomotion/drake/blob/4ee674e7931527df838bd33e79cf2f4dad57bd20/solvers/branch_and_bound.h#L674
            atol = 2e-2
            rtol = 2e-2
            gap = self.plans[plan]['cost'] - target_cost[plan]
            atol_convergence = gap <= atol
            rtol_convergence = gap / target_cost[plan] <= rtol
            self.assertTrue(
                atol_convergence or rtol_convergence,
                f'Target cost of {target_cost[plan]} is not achieved for '
                + f'terrain_{plan}, with absolute tolerance {atol} and '
                + f'relative tolerance {rtol}.')
