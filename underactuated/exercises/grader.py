import unittest
from gradescope_utils.autograder_utils.json_test_runner import JSONTestRunner

import json

class Grader:

    def __init__(self):
        pass
    
    @staticmethod
    def grade_output(test_cases_list, notebook_locals_list, results_json=None):
        '''Grading the notebook_locals with the provided test_cases'''

        # setup test suite for gradescope
        suite = unittest.TestSuite()
        for test_case, notebook_locals  in zip(test_cases_list, notebook_locals_list):
            test_case_names = unittest.defaultTestLoader.getTestCaseNames(test_case)
            for test_case_name in test_case_names:
                suite.addTest(test_case(test_case_name, notebook_locals))
                
        # run all the tests in the suite
        if not results_json:
            results_json = 'results.json'
            display('hello')
        with open('results.json', 'w') as fh:
            JSONTestRunner(stream=fh).run(suite)

    @staticmethod
    def print_test_results(results_json):
        '''Printing the results.json'''

        # open the json file for reading
        with open(results_json, 'r') as fh:
            result = json.load(fh)

        # print total score
        max_score = sum(test['max_score'] for test in result['tests'])
        print('Total score is {}/{}.'.format(int(result['score']), max_score))

        # print partial scores
        for test in result['tests']:
            print('\nScore for {} is {}/{}.'.format(test['name'], int(test['score']), test['max_score']))

            # print error message if any
            if 'output' in test:
                print('- ' + test['output'])

    @staticmethod
    def notebooks_to_locals_list(notebooks):
        notebook_locals = []
        for notebook in notebooks:
            notebook_locals.append(run_module(notebook))
        return notebook_locals_list