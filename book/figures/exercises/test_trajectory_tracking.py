"""Run trajectory_tracking.py as test (Bazel rt_py_test)."""

import os
import runpy


def test_trajectory_tracking():
    path = os.path.join(os.path.dirname(__file__), "trajectory_tracking.py")
    runpy.run_path(path, run_name="__main__")
