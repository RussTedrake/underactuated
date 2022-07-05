import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestRRTPlanning(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(7)
    @timeout_decorator.timeout(1.)
    def test1_rrt_path(self):
        """Test RRT"""
        # load locals
        RRT = self.notebook_locals['RRT']

        start = np.array([11, 0])
        goal = np.array([6, 8])

        obstacles = [
            np.array([9, 6, 2]),
            np.array([9, 8, 1]),
            np.array([9, 10, 2]),
            np.array([4, 5, 2]),
            np.array([7, 5, 2]),
            np.array([4, 10, 1])
        ]

        bounds = np.array([-2, 15])

        np.random.seed(7)
        rrt = RRT(start=start,
                  goal=goal,
                  bounds=bounds,
                  obstacle_list=obstacles)
        path = rrt.plan()

        self.assertFalse(path is None,
                         'The plan method is not implemented correctly')

        np.testing.assert_array_almost_equal(
            start,
            path[-1],
            err_msg='The start location of the path is not the correct '
            + 'start location')

        np.testing.assert_array_almost_equal(
            goal,
            path[0],
            err_msg='The goal location of the path is not the correct '
            + 'goal location')

        for i in range(len(path) - 1):
            is_in_collision = RRT.collision(RRT.Node(path[i]),
                                            RRT.Node(path[i + 1]), obstacles)
            self.assertFalse(is_in_collision,
                             'The path is colliding with obstacles')

    @weight(7)
    @timeout_decorator.timeout(20.)
    def test2_rrt_star_path(self):
        """Test RRT*"""
        # load locals
        RRTStar = self.notebook_locals['RRTStar']

        start = np.array([11, 0])
        goal = np.array([6, 8])

        obstacles = [
            np.array([9, 6, 2]),
            np.array([9, 8, 1]),
            np.array([9, 10, 2]),
            np.array([4, 5, 2]),
            np.array([7, 5, 2]),
            np.array([4, 10, 1])
        ]

        bounds = np.array([-2, 15])

        rrt_star = RRTStar(start=start,
                           goal=goal,
                           bounds=bounds,
                           obstacle_list=obstacles)

        # Check the choose_parent method
        node1 = rrt_star.Node(np.array([12, 0]))
        node1 = rrt_star.steer(rrt_star.start, node1)
        node1.cost = 10
        node2 = rrt_star.Node(np.array([10, 0]))
        node2 = rrt_star.steer(rrt_star.start, node2)
        node2.cost = 20
        node3 = rrt_star.Node(np.array([5, 8]))
        node3 = rrt_star.steer(rrt_star.start, node3)
        node3.cost = 0
        node4 = rrt_star.Node(np.array([5, 0]))
        rrt_star.node_list = [node1, node2, node3]
        node4 = rrt_star.steer(rrt_star.start, node4)
        near_inds = rrt_star.near_nodes_inds(node4)
        node4 = rrt_star.choose_parent(node4, near_inds)

        self.assertFalse(
            node4.parent is None,
            'The choose_parent method is not implemented correctly')

        self.assertFalse(
            np.linalg.norm(node3.p - node4.parent.p) < 1e-5,
            'The choose_parent method does not check for collisions')

        np.testing.assert_array_almost_equal(
            node4.parent.p,
            node1.p,
            err_msg='The choose_parent method does not seem to'
            + ' select the best parent node')

        # Check the rewiring method
        node4 = rrt_star.steer(rrt_star.start, node4)
        rrt_star.rewire(node4, near_inds)
        np.testing.assert_array_almost_equal(
            node1.parent.p,
            node4.p,
            err_msg='The rewiring method does not rewire the correct nodes')

        node3.cost = 100
        node4 = rrt_star.steer(rrt_star.start, node4)
        rrt_star.rewire(node4, near_inds)
        self.assertFalse(
            np.linalg.norm(node3.parent.p - node4.p) < 1e-5,
            'The rewiring method does not check for collision')

        np.random.seed(7)
        rrt_star = RRTStar(start=start,
                           goal=goal,
                           bounds=bounds,
                           obstacle_list=obstacles,
                           goal_sample_rate=0.0,
                           max_iter=300)
        path, _ = rrt_star.plan()

        np.testing.assert_array_almost_equal(
            start,
            path[-1],
            err_msg='The start location of the path is not the correct '
            + 'start location')

        np.testing.assert_array_almost_equal(
            goal,
            path[0],
            err_msg='The goal location of the path is not the correct '
            + 'goal location')

        for i in range(len(path) - 1):
            is_in_collision = RRTStar.collision(RRTStar.Node(path[i]),
                                                RRTStar.Node(path[i + 1]),
                                                obstacles)
            self.assertFalse(is_in_collision,
                             'The path is colliding with obstacles')

        def path_cost(path):
            total = 0
            for i in range(len(path) - 1):
                total += np.linalg.norm(path[i] - path[i + 1])
            return total

        path_length = path_cost(path)
        self.assertTrue(path_length < 17.5,
                        'Length of the path={} > 17.5'.format(path_length))
