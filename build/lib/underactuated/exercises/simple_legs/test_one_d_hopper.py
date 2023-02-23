import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestOneDHopper(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(3)
    @timeout_decorator.timeout(1.)
    def test_mechanical_energy(self):
        """Test mechanical energy"""
        # retrieve energy function
        mechanical_energy = self.notebook_locals['mechanical_energy']

        # evaluate energy at N random points
        N = 100
        np.random.seed(0)
        state = np.random.randn(4, N)
        delta = np.random.randn(N)
        E = mechanical_energy(state, delta)

        # check size of E
        self.assertFalse(
            isinstance(E, (float, int)),
            msg=f'The function mechanical_energy does not take vectorized '
            + 'inputs.')
        self.assertTrue(
            E.size == N,
            msg=f'Passing an array of shape (4, {N}) for the states and an '
            + f'array of shape ({N},) for delta, got back an array of size '
            + f'{E.size} from the function mechanical_energy. The output array '
            + f'must have size {N} instead.')

        # compare with target values
        E_target = np.array([
            1035.88403634, 3066.05294704, 226.59829248, 1132.19808917,
            4519.60941109, 2571.77722312, 179.75301624, 218.267293,
            1518.71951559, 285.28744414, 126.93879604, 1623.09759602,
            761.6724944, 1439.16285815, 1123.31075613, 1150.29073766,
            1039.18526308, -22.85241161, 2344.88095522, 3406.4444863,
            586.89991222, 194.75259958, 1710.76292998, 985.57408993,
            593.67007518, -156.80729216, 129.76883127, 397.72717689,
            179.93169091, 392.49996925, 784.32597859, 433.87021831,
            -81.97555113, -164.62661332, 1261.32659852, 518.81015917,
            1127.06741275, 171.04599127, 678.92178616, 476.58699247,
            1653.76175357, 111.44797797, -178.80377977, 562.21356062,
            5279.60802163, 321.88033307, -33.49681192, 113.06257362,
            2018.19847436, 147.46771374, 534.50643625, 3271.78401092,
            -21.5353477, 15.30134453, 27.29342195, 963.63275164, 240.49348165,
            5417.05008921, 384.4609627, 78.46055822, 445.4086912, 433.45750051,
            53.56942879, -149.21833514, 1521.20337472, 5268.5937194,
            1737.91194436, 2767.73189444, 520.04810383, 170.84779081,
            688.84932501, 86.93212697, 323.27284373, 786.95926209, 283.83230408,
            1338.02751306, 33.00346265, 1167.80440843, 211.53932223,
            163.70938173, -40.39089787, 149.60356967, 1255.61900843,
            3329.55930887, 1274.00886131, 9255.17733426, 207.51077886,
            171.65671047, 190.63432638, 1693.4769287, 2643.46635344,
            139.37393331, 613.39405767, 111.57079258, 5226.36517526,
            111.22451849, 15.62008105, 251.80124408, 692.01834266, 850.05120716
        ])
        np.testing.assert_array_almost_equal(
            E_target,
            E.flatten(),
            err_msg='The function mechanical_energy does not return the '
            + 'correct values when evaluated at random points.')

    @weight(10)
    @timeout_decorator.timeout(15.)
    def test_preload_controller(self):
        """Test preload controller"""
        # retrieve controller and simulator
        PreloadController = self.notebook_locals['PreloadController']
        simulate = self.notebook_locals['simulate']

        # number of random simulations
        N = 10

        # duration of each simulation, just enough to ensure
        # convergence independently of the initial condition
        duration = 6

        # final window of time in which convergence must be achieved
        tail = 2

        np.random.seed(1)
        for i in range(N):

            # desired hopping height from .5 to 1.5
            h = np.random.rand() + .5

            # initial body height from 0 to 1
            zb0 = np.random.rand()
            initial_state = np.array([zb0, 0, 0, 0])

            # instantiate controller
            preload_controller = PreloadController(h)

            # run simulation
            logger = simulate(initial_state, duration, preload_controller)[1]

            # unpack logger
            time = logger.sample_times()
            zb = logger.data()[0]

            # get body height for the final window of time
            zb_tail = zb[np.where(time >= duration - tail)[0]]

            # ensure that in final 2 seconds of simulation the body
            # is always below h plus a tolerance
            tol = .1
            self.assertTrue(
                max(zb_tail) < h + tol,
                msg=f'Starting from the state {initial_state} and having a '
                + f'target hopping height of {h}, the hopper body still hops '
                + f'above {h} after {duration - tail} seconds.')

            # ensure that in final 2 seconds of simulation the body achieves
            # the hopping height h at least once
            self.assertTrue(
                max(zb_tail) > h - tol,
                msg=f'Starting from the state {initial_state} and having a '
                + f'target hopping height of {h}, the hopper body does not '
                + f'reach the height {h} after {duration - tail} seconds.')
