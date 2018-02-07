import os
import subprocess
import sys
import tempfile

import nbformat

if (len(sys.argv) < 2):
    print("Usage: test_ipynb file")
    exit(-1)
path = os.path.abspath(sys.argv[1])
print(path)

# Logic below is adapted from https://blog.thedataincubator.com/2016/06/testing-jupyter-notebooks/

dirname, __ = os.path.split(path)
os.chdir(dirname)
with tempfile.NamedTemporaryFile(suffix=".ipynb") as fout:
    args = ["jupyter", "nbconvert", "--to", "notebook", "--execute",
      "--ExecutePreprocessor.timeout=60",
      "--output", fout.name, path]
    subprocess.check_call(args)

    fout.seek(0)
    nb = nbformat.read(fout, nbformat.current_nbformat)

errors = [output for cell in nb.cells if "outputs" in cell
                 for output in cell["outputs"]\
                 if output.output_type == "error"]

assert errors == []
