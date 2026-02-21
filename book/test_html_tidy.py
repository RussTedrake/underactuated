from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
BOOK_DIR = ROOT / "book"
TIDY_CONFIG = BOOK_DIR / ".tidy.config"


def _book_html_sources() -> list[Path]:
    files = list(BOOK_DIR.glob("*.html"))
    files.extend(BOOK_DIR.glob("*.html.in"))
    return sorted(files)


@pytest.mark.skipif(shutil.which("tidy") is None, reason="tidy is not installed")
@pytest.mark.parametrize("html_file", _book_html_sources())
def test_html_tidy(html_file: Path) -> None:
    result = subprocess.run(
        [
            "tidy",
            "-config",
            str(TIDY_CONFIG),
            "-output",
            "/dev/null",
            str(html_file),
        ],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
    )
    # tidy returns 0 (clean) or 1 (warnings). Bazel accepts both.
    if result.returncode not in (0, 1):
        pytest.fail(
            "\n\n".join(
                [
                    f"tidy failed for {html_file.relative_to(ROOT)}",
                    f"Exit code: {result.returncode}",
                    f"STDOUT:\n{result.stdout}",
                    f"STDERR:\n{result.stderr}",
                ]
            )
        )
