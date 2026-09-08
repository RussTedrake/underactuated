# Repository Agent Notes

- Always run Python commands (including tests, linting, type checks, scripts, and tooling) using the repository virtual environment at `.venv`.
- Prefer `.venv/bin/python -m <tool>` over system Python or globally installed executables.
- Use the GitHub account `RussTedrake` for this repository; do not use `RussTedrake-walden`.
- See [docs/hosting.md](docs/hosting.md) before working on the live server.
- Edit exercise notebooks in `solutions/notebooks`, then regenerate the public exercises with `.venv/bin/python solutions/install.py`; do not edit generated exercise notebooks directly.
