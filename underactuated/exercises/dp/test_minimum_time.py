import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight


class TestMinimumTime(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(4)
    @timeout_decorator.timeout(1.)
    def test_q1_policy_closed_form(self):
        """Test closed form policy"""
        # note: all prints here go to the output item in the json file
        policy_closed_form = self.notebook_locals['policy_closed_form']
        self.assertEqual(policy_closed_form(q=0, qdot=0), 0,
                         "The optimal control at [q=0.0,qd=0.0] should be 0.")

        qdot = -10.0
        q_s = [5.0, -10.0, 0.0]
        for q in q_s:
            self.assertEqual(
                policy_closed_form(q, qdot), 1, "Incorrect control applied at \
                              [q=%2.2f, qd=%2.2f]." % (q, qdot))

        qdot = 15.0
        for q in q_s:
            self.assertEqual(
                policy_closed_form(q, qdot), -1, "Incorrect control applied at \
                             [q=%2.2f, qd=%2.2f]." % (q, qdot))
