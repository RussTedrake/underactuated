import subprocess
import sys
from pathlib import Path

import pytest


@pytest.mark.parametrize("with_solutions", [False, True])
def test_collection_respects_ignore_without_private_checkout(tmp_path, with_solutions):
    conftest = Path(__file__).resolve().parents[1] / "conftest.py"
    (tmp_path / "conftest.py").write_text(
        conftest.read_text() + '\ncollect_ignore = ["test_ignored.py"]\n'
    )
    (tmp_path / "test_ignored.py").write_text(
        'raise AssertionError("Ignored modules must not be imported")\n'
    )
    (tmp_path / "test_active.py").write_text("def test_active(): pass\n")
    if with_solutions:
        (tmp_path / "solutions").mkdir()

    result = subprocess.run(
        [sys.executable, "-m", "pytest", "--collect-only", "-q", str(tmp_path)],
        cwd=tmp_path,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "test_active.py::test_active" in result.stdout
    assert "1 test collected" in result.stdout
