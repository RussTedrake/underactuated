"""Check that every notebook in the book is executed by some test.

See htmlbook/notebook_coverage.py. Notebooks that cannot run in CI must be
listed in `UNTESTED` with a reason.
"""

from pathlib import Path

import pytest
from htmlbook.notebook_coverage import (
    assert_book_notebooks_are_tested,
    assert_exercise_notebooks_are_tested,
    assert_untested_list_is_current,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
BOOK = REPO_ROOT / "book"
SOLUTIONS_NOTEBOOKS = REPO_ROOT / "solutions" / "notebooks"

# Paths are relative to book/.
UNTESTED = {
    "contact/hybrid.ipynb": "Waiting for Hybrid TrajOpt to land in Drake.",
    "figures/lcp_cart.ipynb": "Needs plotly, which is not worth a dependency.",
}


def test_book_notebooks_are_tested() -> None:
    assert_book_notebooks_are_tested(BOOK, UNTESTED)


def test_untested_list_is_current() -> None:
    assert_untested_list_is_current(BOOK, UNTESTED)


@pytest.mark.skipif(
    not SOLUTIONS_NOTEBOOKS.is_dir(), reason="needs the solutions checkout"
)
def test_exercise_notebooks_are_tested() -> None:
    assert_exercise_notebooks_are_tested(BOOK, SOLUTIONS_NOTEBOOKS)
