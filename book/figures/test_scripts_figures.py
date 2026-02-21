from __future__ import annotations

import contextlib
import os
import runpy
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


@contextlib.contextmanager
def _chdir(path: Path):
    old = Path.cwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(old)


def _run(script_rel: str) -> None:
    with tempfile.TemporaryDirectory(prefix="script_cwd_") as temp_cwd:
        with _chdir(Path(temp_cwd)):
            runpy.run_path(str(ROOT / script_rel), run_name="__main__")


def test_graphical_analysis_script() -> None:
    _run("book/figures/exercises/graphical_analysis.py")


def test_trajectory_tracking_script() -> None:
    _run("book/figures/exercises/trajectory_tracking.py")


def test_stereographic_script() -> None:
    _run("book/figures/stereographic.py")


def test_vanderpol_staircase_script() -> None:
    _run("book/figures/vanderpol_staircase.py")
