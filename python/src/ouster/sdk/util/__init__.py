"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

Generic util module
"""
# flake8: noqa: F401 (unused imports)

from .metadata import resolve_metadata
from .metadata import resolve_metadata_multi
from .metadata import firmware_version      # TODO[UN]: this function should
                                            # probably should be moved to
                                            # its own file.

from .parsing import default_scan_fields    # type: ignore
from .parsing import scan_to_packets        # type: ignore
from .parsing import resolve_field_types    # type: ignore
from .parsing import PacketFormat           # type: ignore
from .parsing import ColHeader              # type: ignore
from .parsing import FusaDualFormat         # type: ignore

from .extrinsics import resolve_extrinsics      # type: ignore
from .extrinsics import _parse_extrinsics_file  # type: ignore
from .extrinsics import img_aspect_ratio        # type: ignore

from .progress_bar import progressbar

from .forward_slicer import ForwardSlicer
