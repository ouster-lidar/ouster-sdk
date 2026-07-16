"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

This module provides generic utility functions and helpers for working with Ouster lidar data.
"""
# flake8: noqa: F401 (unused imports)

from .metadata import resolve_metadata
from .metadata import resolve_metadata_multi

from .parsing import frame_to_packets        # type: ignore
from ouster.sdk._deprecation import deprecated_alias
# ``scan_to_packets`` was renamed to ``frame_to_packets``. Emit a deprecation
# warning on use rather than aliasing it silently.
deprecated_alias("scan_to_packets", "frame_to_packets", frame_to_packets, globals(), "1.0")

from .extrinsics import img_aspect_ratio        # type: ignore

from .progress_bar import progressbar
from .progress_bar import ProgressBar

from .forward_slicer import ForwardSlicer

from ouster.sdk._bindings.client import resolve_field_types
