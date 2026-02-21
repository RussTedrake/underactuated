# Pants build (Python tests and checks)

This repo uses [Pants](https://pantsbuild.org/) for Python testing and HTML checks, alongside or instead of Bazel.

## Setup

1. **Install Pants**  
   Pants 2.x is **no longer on PyPI**; the last pip-installable version was 2.17.1 and it only supports **Python 3.7–3.9**. So with Python 3.10+ (including your venv’s 3.13), use one of:
   - **Homebrew (recommended):** `brew install pantsbuild/tap/pants`
   - **Launcher script:** `curl -fsSL https://static.pantsbuild.org/setup/get-pants.sh | bash` (installs to `~/.local/bin`)
   Both install the `pants` launcher only and do not touch your Python or venv.

2. **Generate the lockfile** (first time and when changing deps):
   ```bash
   pants generate-lockfiles
   ```
   This creates `python-default.lock` from **Poetry** (`pyproject.toml`). Pants does not use `poetry.lock` or `requirements-bazel.txt`; it reads `[tool.poetry.dependencies]` (and dev deps) and resolves them itself. Bazel can continue to use `requirements-bazel.txt` from your existing Poetry export if you still run Bazel.

3. **Optional: drake_models** (for underactuated examples that use `package://drake_models/...`):
   ```bash
   ./setup/fetch_drake_models.sh
   ```
   Then set `DRAKE_MODELS_PATH` to `third_party/drake_models` if your code expects it elsewhere.

4. **Optional: tidy** (for HTML tidy test):
   - macOS: `brew install tidy-html5`
   - Ubuntu: `apt install tidy`

## Commands

- **Run all Python and book tests:**
  ```bash
  pants test ::
  ```

- **Run only underactuated exercise tests:**
  ```bash
  pants test underactuated::
  ```

- **Run only book/htmlbook and figures tests:**
  ```bash
  pants test book/htmlbook:: book/figures::
  ```

- **Run HTML tidy + link-check tests:**
  ```bash
  ./pants test book/htmlbook/tools/tidy: book/htmlbook/tools/html:
  ```

- **List targets:**
  ```bash
  pants list ::
  ```

## Solutions

The `solutions/` directory is gitignored (separate repo). If you clone it (e.g. `underactuated-solutions`), you can add a `BUILD` there and add `solutions/...` to the `all_python_tests` dependency in the root `BUILD` to run solution tests with Pants.

## Notebooks

Notebook (`.ipynb`) execution is not yet wired as Pants tests. The Bazel `rt_ipynb_test` targets are still in `BUILD.bazel`; you can run those with Bazel or add a Pants script/plugin later to run nbconvert and execute notebooks.
