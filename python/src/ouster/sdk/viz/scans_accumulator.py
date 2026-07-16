"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

Deprecated module. Use ouster.sdk.viz.frames_accumulator instead.
"""
from ouster.sdk._deprecation import warn_deprecated
warn_deprecated(
    "ouster.sdk.viz.scans_accumulator is deprecated. "
    "Use ouster.sdk.viz.frames_accumulator instead. "
    "The last supported version for this will be 1.0.",
    stacklevel=2
)
from ouster.sdk.viz.frames_accumulator import *  # noqa: F401, F403, E402
