"""Run graphical_analysis.py as test (Bazel rt_py_test)."""

import os
import runpy


def test_graphical_analysis():
    path = os.path.join(os.path.dirname(__file__), "graphical_analysis.py")
    runpy.run_path(path, run_name="__main__")
