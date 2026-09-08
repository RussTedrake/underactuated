from __future__ import annotations

import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent

# Deepnote projects are retired and will not receive this package release.
# Remove this exclusion when the Colab migration updates htmlbook, which deletes
# the obsolete test while also removing the legacy textbook link helpers.
collect_ignore = ["book/htmlbook/test_check_deepnote_requirements.py"]


def pytest_configure() -> None:
    os.environ.setdefault("MPLBACKEND", "Agg")
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
    os.environ.setdefault("XDG_CACHE_HOME", "/tmp")
    Path("/tmp/matplotlib").mkdir(parents=True, exist_ok=True)
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    if str(REPO_ROOT / "book") not in sys.path:
        sys.path.insert(0, str(REPO_ROOT / "book"))


def pytest_ignore_collect(collection_path, config):  # type: ignore[no-untyped-def]
    solutions_dir = REPO_ROOT / "solutions"
    if solutions_dir.exists():
        return None
    try:
        return Path(collection_path).resolve().is_relative_to(solutions_dir)
    except Exception:
        return None
