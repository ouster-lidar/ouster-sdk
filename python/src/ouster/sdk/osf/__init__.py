"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

This module provides an API for working with OSF (Ouster Sensor Format) files.
It includes tools for reading, writing, slicing, encoding, and modifying OSF files and their metadata.
"""
# flake8: noqa (unused imports)

import importlib

from ouster.sdk._kwarg_aliases import install_osf_kwarg_aliases

install_osf_kwarg_aliases(importlib.import_module("ouster.sdk._bindings.osf"))

from ouster.sdk._bindings.osf import Writer
from ouster.sdk._bindings.osf import AsyncWriter
from ouster.sdk._bindings.osf import OsfFrameSetSource
from ouster.sdk._bindings.osf import Reader
from ouster.sdk._bindings.osf import LidarSensor
from ouster.sdk._bindings.osf import LidarFrameStream

from ouster.sdk._bindings.osf import slice_and_cast
from ouster.sdk._bindings.osf import dump_metadata
from ouster.sdk._bindings.osf import backup_osf_file_metablob
from ouster.sdk._bindings.osf import restore_osf_file_metablob
from ouster.sdk._bindings.osf import osf_file_modify_metadata
from ouster.sdk._bindings.osf import Encoder, PngLidarFrameEncoder, ZPngLidarFrameEncoder
from ouster.sdk._bindings.osf import OsfDropFrameError

from ouster.sdk._deprecation import deprecated_alias
deprecated_alias("OsfDropScanError", "OsfDropFrameError", OsfDropFrameError, globals(), "1.0")
deprecated_alias("OsfScanSource", "OsfFrameSetSource", OsfFrameSetSource, globals(), "1.0")
deprecated_alias("LidarScanStream", "LidarFrameStream", LidarFrameStream, globals(), "1.0")
deprecated_alias("PngLidarScanEncoder", "PngLidarFrameEncoder", PngLidarFrameEncoder, globals(), "1.0")
deprecated_alias("ZPngLidarScanEncoder", "ZPngLidarFrameEncoder", ZPngLidarFrameEncoder, globals(), "1.0")
