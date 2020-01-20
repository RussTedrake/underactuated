import os
import subprocess
import sys
import tempfile

from nbconvert.preprocessors import ExecutePreprocessor
import nbformat


def main():
    if len(sys.argv) < 2:
        print("Usage: test_ipynb file")
        exit(-1)
    path = os.path.abspath(sys.argv[1])
    print(path)

    # Logic below is adapted from TRI Anzu code, which just follows:
    # https://nbconvert.readthedocs.io/en/latest/execute_api.html
    # N.B. `os.environ["IPYTHONDIR"]` should be set if this is to not write to
    # $HOME.
    dirname, __ = os.path.split(path)
    os.chdir(dirname)
    with open(path) as fout:
        nb = nbformat.read(fout, as_version=4)
    ep = ExecutePreprocessor(
        timeout=60,
        kernel_name="python{}".format(sys.version_info.major))
    # Any errors encountered will raise a `CellExecutionError`.
    ep.preprocess(nb, {"metadata": {"path": os.getcwd()}})


if __name__ == "__main__":
    main()
