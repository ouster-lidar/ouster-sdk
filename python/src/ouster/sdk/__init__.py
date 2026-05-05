"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

Ouster SDK Python package.
Provides APIs for working with Ouster lidar sensors, such as sensor configuration, reading, visualizing, and processing data.
"""
# flake8: noqa (unused imports)

from importlib.metadata import version as _get_version
__version__ = _get_version("ouster-sdk")

from .open_source import open_source, open_packet_source
from .open_source import SourceURLException

from ._bindings.client import populate_extrinsics as _populate_extrinsics
