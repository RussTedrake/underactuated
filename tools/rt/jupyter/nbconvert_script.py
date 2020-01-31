# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

import os
import re
import sys

from nbconvert.exporters import PythonExporter
from nbconvert.writers import StdoutWriter


def main(notebook_filename):
    resources = {}
    basename = os.path.basename(notebook_filename)
    resources["unique_key"] = basename[:basename.rfind(".")]
    exporter = PythonExporter()
    output, resources = exporter.from_filename(notebook_filename,
                                               resources=resources)
    writer = StdoutWriter()
    write_results = writer.write(output, resources)


if __name__ == "__main__":
    sys.argv[0] = re.sub(r"_script\.py$", "", sys.argv[0])
    if len(sys.argv) != 2:
        print("Notebook filename is missing", file=sys.stderr)
        sys.exit(1)
    main(sys.argv[1])
