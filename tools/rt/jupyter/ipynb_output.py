# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

import json
import sys

# Simple script to check that the output fields of the notebook are empty, to
# prevent accidentally committing this metadata to the repo.

def main(filename):
    ipynb = json.load(open(filename))
    for cell in ipynb['cells']:
        if "outputs" in cell and cell['outputs']:
            print("The notebook " + filename + 
                  " has output/metadata in the file.  You must clear it " +
                  "before committing (or set the ipynboutput attribute to " +
                  "False in your BUILD file).")
            exit(2)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Notebook filename is missing", file=sys.stderr)
        sys.exit(1)
    main(sys.argv[1])
