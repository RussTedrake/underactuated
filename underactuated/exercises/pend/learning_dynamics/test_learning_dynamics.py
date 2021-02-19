import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight


class TestLearningDynamics(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(5)
    @timeout_decorator.timeout(10.)
    def test_final_loss(self):
        """Test dynamics model training"""
        # note: all prints here go to the output item in the json file

        # the loss must be close to the correct value
        final_loss = self.notebook_locals['final_loss']
        self.assertLessEqual(
            final_loss,
            0.38,
            msg='Final loss is less than upper bound for the correct value')
        self.assertGreaterEqual(
            final_loss,
            0.35,
            msg='Final loss is greater than lower bound for the correct value')
