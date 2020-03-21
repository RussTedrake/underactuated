import unittest
import timeout_decorator
from gradescope_utils.autograder_utils.decorators import weight
import numpy as np


class TestShootingVsTranscription(unittest.TestCase):

    def __init__(self, test_name, notebook_locals):
        super().__init__(test_name)
        self.notebook_locals = notebook_locals

    @weight(3)
    @timeout_decorator.timeout(1.)
    def test_lost_bits_shooting(self):
        """Test number of bits lost with direct shooting"""
        # get list of lost bits
        lost_bits = self.notebook_locals['lost_bits_shooting']

        # check list length and types
        self.assertEqual(
            len(lost_bits), 45,
            'The list lost_bits_shooting must have length equal' + 'to 45.')
        bits_type = [isinstance(bits, (float, int)) for bits in lost_bits]
        self.assertTrue(
            all(bits_type), 'Some entries in lost_bits_shooting are neither' +
            'floats nor ints.')

        # check values
        lost_bits_shooting = [
            0.0, 0.9543364613711909, 1.8123366596726525, 2.845325160323255,
            3.997099761655346, 5.181146076477944, 6.367665432089994,
            7.549745045378025, 8.727297345452408, 9.901650046513046,
            11.074027866040504, 12.245271423678894, 13.415892162590488,
            14.586181089218373, 15.75629682473095, 16.92632331212132,
            18.096304057604645, 19.266261289075352, 20.436206266052793,
            21.606144677268965, 22.776079410190473, 23.94601195116546,
            25.115943085840197, 26.285873247157944, 27.455802687733357,
            28.625731565680532, 29.795659987345097, 30.965588028826133,
            32.13551574866848, 33.30544319406715, 34.47537039717343,
            35.64529738697145, 36.81522421237407, 37.98515087701531,
            39.15507737686514, 40.32500371458269, 41.49492968101353,
            42.6648551872069, 43.83478032928076, 45.00470555237002,
            46.174631383871656, 47.34456378440541, 48.52985492074265,
            49.72208993747556, 50.8608777068896
        ]
        for i, bits in enumerate(lost_bits):
            self.assertAlmostEqual(bits,
                                   lost_bits_shooting[i],
                                   msg='lost_bits_shooting are incorrect.')

    @weight(4)
    @timeout_decorator.timeout(1.)
    def test_J_star_N_transcription(self):
        """Test value function direct transcription"""
        # get list of costs to go
        J_star_N = self.notebook_locals['J_star_N_transcription']

        # check list length and types
        self.assertEqual(
            len(J_star_N), 100,
            'The list J_star_N_transcription must have length' +
            'equal to 100.')
        J_star_type = [isinstance(J, (float, int)) for J in J_star_N]
        self.assertTrue(
            all(J_star_type),
            'Some entries in J_star_N_transcription are neither' +
            'floats nor ints.')

        # check values
        J_star_N_transcription_30 = [
            10.75, 18.506849315068486, 24.221925133689844, 26.538390153365242,
            27.14309163681062, 27.243582216689386, 27.24200007642889,
            27.231499085690622, 27.22561231689258, 27.223217461345868,
            27.222393413120635, 27.222143703386458, 27.222076887248058,
            27.222061591713526, 27.222058929530345, 27.22205878066236,
            27.22205892600524, 27.222059017397388, 27.22205905444769,
            27.222059066651674, 27.222059070055884, 27.222059070835737,
            27.222059070957886, 27.22205907095374, 27.222059070939768,
            27.22205907093212, 27.222059070929042, 27.222059070927997,
            27.222059070927685, 27.222059070927596
        ]
        for i, J in enumerate(J_star_N):
            self.assertAlmostEqual(J,
                                   J_star_N_transcription_30[min(i, 29)],
                                   msg='J_star_N_transcription are incorrect.')

    @weight(3)
    @timeout_decorator.timeout(1.)
    def test_lost_bits_transcription(self):
        """Test number of bits lost with direct transcription"""
        # get list of lost bits
        lost_bits = self.notebook_locals['lost_bits_transcription']

        # check list length and types
        self.assertEqual(
            len(lost_bits), 100,
            'The list lost_bits_transcription must have length' +
            'equl to 100.')
        bits_type = [isinstance(bits, (float, int)) for bits in lost_bits]
        self.assertTrue(
            all(bits_type),
            'Some entries in lost_bits_transcription are neither' +
            'floats nor ints.')

        # check values
        lost_bits_transcription_30 = [
            6.659823518580827, 7.566530101329372, 7.989788581807347,
            8.126275184924792, 8.156165618892503, 8.158803371775676,
            8.1571939291375, 8.155956959505353, 8.155384990230832,
            8.155170750799739, 8.155101341468276, 8.155081672171026,
            8.155076924745707, 8.155076049611791, 8.15507599036841,
            8.155076036096093, 8.155076066576788, 8.155076079192476,
            8.155076083345309, 8.1550760844654, 8.155076084693125,
            8.155076084711247, 8.155076084698281, 8.155076084688659,
            8.15507608468425, 8.15507608468261, 8.155076084682092,
            8.15507608468194, 8.155076084681896, 8.155076084681886
        ]
        for i, bits in enumerate(lost_bits):
            self.assertAlmostEqual(bits,
                                   lost_bits_transcription_30[min(i, 29)],
                                   msg='lost_bits_transcription are incorrect.')

    @weight(4)
    @timeout_decorator.timeout(1.)
    def test_J_star_N_riccati(self):
        """Test value function Riccati recursion"""
        # get list of costs to go
        J_star_N = self.notebook_locals['J_star_N_riccati']

        # check list length and types
        self.assertEqual(
            len(J_star_N), 1000,
            'The list J_star_N_riccati must have length equal' + 'to 1000.')
        J_star_type = [isinstance(J, (float, int)) for J in J_star_N]
        self.assertTrue(
            all(J_star_type),
            'Some entries in J_star_N_riccati are neither floats' + 'nor ints.')

        # check values
        J_star_N_riccati_30 = [
            10.75, 18.506849315068493, 24.221925133689837, 26.538390153365242,
            27.14309163681062, 27.243582216689365, 27.242000076428884,
            27.231499085690622, 27.225612316892573, 27.223217461345843,
            27.222393413120585, 27.222143703386408, 27.222076887248043,
            27.222061591713505, 27.22205892953035, 27.22205878066236,
            27.222058926005232, 27.222059017397356, 27.22205905444766,
            27.222059066651646, 27.222059070055877, 27.22205907083573,
            27.222059070957876, 27.222059070953733, 27.22205907093976,
            27.22205907093211, 27.22205907092905, 27.222059070928,
            27.222059070927678, 27.222059070927585
        ]
        for i, J in enumerate(J_star_N):
            self.assertAlmostEqual(J,
                                   J_star_N_riccati_30[min(i, 29)],
                                   msg='J_star_N_riccati are incorrect.')
