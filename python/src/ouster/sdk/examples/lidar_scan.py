"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

Deprecated module. Use ouster.sdk.examples.lidar_frame instead.
"""
from ouster.sdk._deprecation import warn_deprecated
warn_deprecated(
    "ouster.sdk.examples.lidar_scan is deprecated. "
    "Use ouster.sdk.examples.lidar_frame instead. "
    "The last supported version for this will be 1.0.",
    stacklevel=2
)
from ouster.sdk.examples.lidar_frame import *  # noqa: F401, F403, E402
