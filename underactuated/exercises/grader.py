import unittest
from gradescope_utils.autograder_utils.json_test_runner import JSONTestRunner
import json
from runpy import run_path
import nbformat
from nbconvert import PythonExporter
import os

grader_throws = False


def set_grader_throws(val):
    global grader_throws
    grader_throws = val


class Grader:

    def __init__(self):
        pass

    @staticmethod
    def grade_from_notebooks(test_cases_list, notebook_ipynb_list,
                             results_json):
        """
        Running notebooks in notebook_ipynb_list and evaluating
        them on test_cases_list. Result is written into results_json
        """
        try:
            notebook_locals_list = []
            for notebook_ipynb in notebook_ipynb_list:
                notebook_locals_list.append(
                    Grader.locals_from_notebook(notebook_ipynb))
        except Exception as e:
            Grader.global_fail_with_error_message(
                "Exception when running file: " + notebook_ipynb + ', '
                + str(e), results_json)
            raise

        # Grade notebook_locals_list on test_cases_list
        Grader.grade_output(test_cases_list, notebook_locals_list, results_json)

    @staticmethod
    def grade_output(test_case_list, notebook_locals_list, results_json):
        """Grading the notebook_locals with the provided test_cases"""
        # setup test suite for gradescope
        suite = unittest.TestSuite()
        for test_case, notebook_locals in zip(test_case_list,
                                              notebook_locals_list):
            test_case_names = unittest.defaultTestLoader.getTestCaseNames(
                test_case)
            for test_case_name in test_case_names:
                suite.addTest(test_case(test_case_name, notebook_locals))

        # run all the tests in the suite
        with open(results_json, 'w') as fh:
            JSONTestRunner(stream=fh).run(suite)

    @staticmethod
    def locals_from_notebook(notebook_ipynb):
        """Read, run, return locals of notebook"""
        banned_commands = ['HTML']

        ipynb = json.load(open(notebook_ipynb))

        for cell in ipynb['cells']:
            # erase test cells, this is optional and useful for debugging
            # to avoid recursions when developing
            if any('## TEST ##' in line for line in cell['source']):
                cell['source'] = []
            # filter out all the lines with banned commands
            if banned_commands is not None:
                cell['source'] = [
                    line for line in cell['source']
                    if not any(command in line for command in banned_commands)
                ]

        exporter = PythonExporter()
        source, meta = exporter.from_notebook_node(
            nbformat.reads(json.dumps(ipynb), nbformat.NO_CONVERT))
        with open('./cleaned_notebook.py', 'w') as fh:
            fh.write(source)

        notebook_locals = run_path('cleaned_notebook.py')
        os.system('rm cleaned_notebook.py')
        return notebook_locals

    @staticmethod
    def global_fail_with_error_message(msg, results_json):
        """Error message if no specific"""
        results = {"score": 0.0, "output": msg}

        with open(results_json, 'w') as f:
            f.write(
                json.dumps(results,
                           indent=4,
                           sort_keys=True,
                           separators=(',', ': '),
                           ensure_ascii=True))

    @staticmethod
    def print_test_results(results_json):
        """Printing the results.json"""
        # open the json file for reading
        with open(results_json, 'r') as fh:
            result = json.load(fh)

        # print total score
        max_score = sum(test['max_score'] for test in result['tests'])
        print('Total score is {}/{}.'.format(int(result['score']), max_score))

        if grader_throws and int(result['score']) != max_score:
            raise RuntimeError("Grader did not award full points.")

        # print partial scores
        for test in result['tests']:
            print('\nScore for {} is {}/{}.'.format(test['name'],
                                                    int(test['score']),
                                                    test['max_score']))

            # print error message if any
            if 'output' in test:
                print('- ' + test['output'])
