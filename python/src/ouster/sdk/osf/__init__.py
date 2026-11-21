"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

This module provides an API for working with OSF (Ouster Sensor Format) files.
It includes tools for reading, writing, slicing, encoding, and modifying OSF files and their metadata.
"""
# flake8: noqa (unused imports)

import importlib

from ouster.sdk._bindings.osf import Writer
from ouster.sdk._bindings.osf import AsyncWriter
from ouster.sdk._bindings.osf import OsfFrameSetSource
from ouster.sdk._bindings.osf import Reader
from ouster.sdk._bindings.osf import LidarSensor
from ouster.sdk._bindings.osf import CameraSensor
from ouster.sdk._bindings.osf import LidarFrameStream

from ouster.sdk._bindings.osf import slice_and_cast
from ouster.sdk._bindings.osf import dump_metadata
from ouster.sdk._bindings.osf import backup_osf_file_metablob
from ouster.sdk._bindings.osf import restore_osf_file_metablob
from ouster.sdk._bindings.osf import osf_file_modify_metadata
from ouster.sdk._bindings.osf import Encoder, FrameEncoder, PngFrameEncoder, ZPngFrameEncoder
from ouster.sdk._bindings.osf import OsfDropFrameError

from ouster.sdk._deprecation import deprecated_alias

# The frame encoders handle camera frames as well as lidar frames, so they lost
# the "Lidar" in their names. Keep the old names importable from here.
deprecated_alias("LidarFrameEncoder", "FrameEncoder", FrameEncoder, globals(), "2.0")
deprecated_alias("PngLidarFrameEncoder", "PngFrameEncoder", PngFrameEncoder, globals(), "2.0")
deprecated_alias("ZPngLidarFrameEncoder", "ZPngFrameEncoder", ZPngFrameEncoder, globals(), "2.0")
