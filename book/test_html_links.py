from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
CHECK_SCRIPT = ROOT / "book/htmlbook/tools/html/check_html_links_exist.py"

# Mirrors rt_html_test targets from book/BUILD.bazel.
BOOK_HTML_FILES = (
    "intro.html",
    "pend.html",
    "acrobot.html",
    "simple_legs.html",
    "humanoids.html",
    "stochastic.html",
    "dp.html",
    "lqr.html",
    "lyapunov.html",
    "trajopt.html",
    "planning.html",
    "feedback_motion_planning.html",
    "policy_search.html",
    "robust.html",
    "output_feedback.html",
    "limit_cycles.html",
    "contact.html",
    "sysid.html",
    "state_estimation.html",
    "rl_policy_search.html",
    "drake.html",
    "multibody.html",
    "optimization.html",
    "playbook.html",
    "misc.html",
)


@pytest.mark.parametrize("html_name", BOOK_HTML_FILES)
def test_html_links_exist(html_name: str) -> None:
    html_path = ROOT / "book" / html_name
    result = subprocess.run(
        [
            sys.executable,
            str(CHECK_SCRIPT),
            "--cwd",
            str(ROOT),
            str(html_path),
        ],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        pytest.fail(
            "\n\n".join(
                [
                    f"HTML link check failed for {html_name}",
                    f"Exit code: {result.returncode}",
                    f"STDOUT:\n{result.stdout}",
                    f"STDERR:\n{result.stderr}",
                ]
            )
        )
