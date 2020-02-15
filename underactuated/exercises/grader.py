import unittest
from gradescope_utils.autograder_utils.json_test_runner import JSONTestRunner
import dill
import json

class Grader:

    def __init__(self):
        pass

    @staticmethod
    def grade_from_dills(notebook_dill_list, test_cases_list, results_json, notebook_ipynb_list=None):
        '''Reading notebook_locals from dills and grading on the corresponding test_cases_list. 
        Optional check for existence of student notebooks in notebook_ipynb_list.
        Result is written into results_json'''

         # Read in all dill files into notebook_locals_list
        try:
            notebook_locals_list = []
            for notebook_dill in notebook_dill_list:
                with open(notebook_dill, "rb") as dill_file:
                    notebook_locals_list.append(dill.load(dill_file))
        except Exception as e:
            Grader.global_fail_with_error_message("Missing file: " + notebook_dill + ', ' + str(e), results_json)
            raise

        # If there are notebooks to be submitted, read in all notebook files into notebook_list
        if notebook_ipynb_list:
            try:
                notebook_list = []
                for notebook_ipynb in notebook_ipynb_list:
                    with open(notebook_ipynb, "rb") as notebook_file:
                        notebook_list.append(json.load(notebook_file))
            except Exception as e:
                Grader.global_fail_with_error_message("Missing file: " + notebook_ipynb + ', ' + str(e), results_json)
                raise

        # Grade notebook_locals_list on test_cases_list
        Grader.grade_output(test_cases_list, notebook_locals_list, results_json)


    @staticmethod
    def grade_output(test_case_list, notebook_locals_list, results_json=None):
        '''Grading the notebook_locals with the provided test_cases'''

        # setup test suite for gradescope
        suite = unittest.TestSuite()
        for test_case, notebook_locals in zip(test_case_list, notebook_locals_list):
            test_case_names = unittest.defaultTestLoader.getTestCaseNames(
                test_case)
            for test_case_name in test_case_names:
                suite.addTest(test_case(test_case_name, notebook_locals))

        # run all the tests in the suite
        if not results_json:
            results_json = 'results.json'
        with open(results_json, 'w') as fh:
            JSONTestRunner(stream=fh).run(suite)


    @staticmethod
    def global_fail_with_error_message(msg, results_json):
        '''Error message if no specific'''
        results = {"score": 0.0,
                "output": msg}

        with open(results_json, 'w') as f:
            f.write(json.dumps(results,
                            indent=4,
                            sort_keys=True,
                            separators=(',', ': '),
                            ensure_ascii=True))

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
            print(
                '\nScore for {} is {}/{}.'.format(test['name'], int(test['score']), test['max_score']))

            # print error message if any
            if 'output' in test:
                print('- ' + test['output'])
