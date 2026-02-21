"""Run stereographic.py as test (Bazel rt_py_test)."""

import os
import runpy


def test_stereographic():
    path = os.path.join(os.path.dirname(__file__), "stereographic.py")
    runpy.run_path(path, run_name="__main__")
