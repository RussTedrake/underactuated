import warnings

from underactuated.meshcat_utils import *

warnings.warn(
    "underactuated.meshcat_cpp_utils has been renamed to underactuated.meshcat_utils. This shim will be removed after 2023-05-31.",
    DeprecationWarning,
    stacklevel=2)
