# Repository Agent Notes

- Always run Python commands (including tests, linting, type checks, scripts, and tooling) using the repository virtual environment at `.venv`.
- Prefer `.venv/bin/python -m <tool>` over system Python or globally installed executables.
- See [docs/hosting.md](docs/hosting.md) for the CSAIL hosting environment and SSH access path before working on deployment or server configuration.
- Exercise notebooks in `book/<chapter>/exercises/` are generated from the local, private `solutions/notebooks/<chapter>/` checkout. Edit those source notebooks, preserve the `remove` and `empty` redaction tags, then run `.venv/bin/python solutions/install.py` to regenerate the public copies. Commit the solutions changes before installation so `solutions_sha.txt` records their revision; verify that regeneration does not expose answer cells.
