## Requirements management with Poetry

```
pip3 install poetry poetry-plugin-export
poetry install --with=dev,docs
```
(in a virtual environment) to install the requirements.

## Bazel currently uses requirements-bazel.txt, which we generate from poetry

If the poetry dependencies change in `pyproject.toml`, then run
```
poetry lock && ./book/htmlbook/PoetryExport.sh
```
to synchronize the requirements-bazel.txt file.

Hopefully [direct poetry
support](https://github.com/bazelbuild/rules_python/issues/340) in bazel will land soon, or I can use [rules_python_poetry](https://github.com/AndrewGuenther/rules_python_poetry) directly; but it looks like it will still require poetry to fix [their issue](# https://github.com/python-poetry/poetry-plugin-export/issues/176).

## Please install the pre-commit hooks

```
pip3 install pre-commit
pre-commit install
```

## Autoflake for notebooks

`autoflake`` is not officially supported by nbqa because it has some risks:
https://github.com/nbQA-dev/nbQA/issues/755 But it can be valuable to run it
manually and check the results.

```
nbqa autoflake --remove-all-unused-imports --in-place .
```

## To Run the Unit Tests

Install the prerequisites:
```bash
bash setup/.../install_prereqs.sh
```

Make sure that you have done a recursive checkout in this repository, or have run

```bash
git submodule update --init --recursive
```
Then run
```bash
bazel test //...
```

## Updating dependencies

Bazel currently uses requirements-bazel.txt, which we generate from poetry

To generate it, run
```
poetry lock && ./book/htmlbook/PoetryExport.sh
```

## To update the pip wheels

Update the version number in `pyproject.toml`, and the drake version, then from
the root directory, run:
```
rm -rf dist/*
poetry publish --build && cd book && ./Deepnote.sh
```
(Use `poetry config pypi-token.pypi <token>` once first)

# To update the Docker image (and pip wheels)

It's good form to update the pip wheels first (so that the Docker contains the
latest pip dependencies):
```
rm -rf dist/*
poetry publish --build
./book/Deepnote_docker.sh
cd book && ./Deepnote.sh
```
And make sure to follow the printed instructions to build the image once on
deepnote. The run a few notebooks on deepnote to convince yourself you haven't
broken anything.

## Building the documentation

You will need to install `sphinx`:
```
poetry install --with docs
pip3 install sphinx myst-parser sphinx_rtd_theme
```

From the root directory, run
```
rm -rf book/python && sphinx-build -M html underactuated /tmp/underactuated_doc && cp -r /tmp/underactuated_doc/html book/python
```
Note that the website will only install the dependencies in the `docs` group, so
`poetry install --only docs` must obtain all of the relevant dependencies.



## To debug a notebook with a local build of Drake

There are several approaches, but perhaps easiest is to just add a few lines at the top of the notebook:
```
import sys
import os

python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
drake_path = os.path.expanduser(f"~/drake-install/lib/{python_version}/site-packages")
if drake_path not in sys.path:
    sys.path.insert(0, drake_path)

import pydrake
print(f"Using pydrake from: {pydrake.__file__}")
```
