"""Prefetch the remote packages used by tests into Drake's cache.

CI runs `python -m setup.prefetch_remotes` from the repository root before
pytest; see docs/testing.md. In the pip job, this imports the installed wheel.
"""

import sys
from pathlib import Path

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant

from underactuated.utils import ConfigureParser

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "book"))
from htmlbook.drake_remotes import prefetch_remotes  # noqa: E402

if __name__ == "__main__":
    parser = Parser(MultibodyPlant(0.0))
    ConfigureParser(parser)
    packages = parser.package_map()
    remotes = ["drake_models", "spot_description"]
    assert all(packages.Contains(name) for name in remotes)
    prefetch_remotes(packages, remotes)
