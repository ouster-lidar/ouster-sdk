"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

Deprecated module. Use ouster.sdk.core.masked_frame_set_source instead.
"""
from ouster.sdk._deprecation import warn_deprecated
warn_deprecated(
    "ouster.sdk.core.masked_scan_source is deprecated. "
    "Use ouster.sdk.core.masked_frame_set_source instead. "
    "The last supported version for this will be 1.0.",
    stacklevel=2
)
from ouster.sdk.core.masked_frame_set_source import *  # noqa: F401, F403, E402
