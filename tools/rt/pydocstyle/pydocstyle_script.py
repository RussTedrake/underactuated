# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

import re
import sys

from pydocstyle.cli import main

if __name__ == "__main__":
    sys.argv[0] = re.sub(r"_script\.py$", "", sys.argv[0])
    sys.exit(main())
