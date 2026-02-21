"""Run vanderpol_staircase.py as test (Bazel rt_py_test)."""

import os
import runpy


def test_vanderpol_staircase():
    path = os.path.join(os.path.dirname(__file__), "vanderpol_staircase.py")
    runpy.run_path(path, run_name="__main__")
