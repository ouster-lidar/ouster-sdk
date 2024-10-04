"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

API to work with OSF files
"""
# flake8: noqa (unused imports)

from ouster.sdk._bindings.osf import Reader
from ouster.sdk._bindings.osf import MessageRef
from ouster.sdk._bindings.osf import MetadataStore
from ouster.sdk._bindings.osf import MetadataEntry
from ouster.sdk._bindings.osf import LidarSensor
from ouster.sdk._bindings.osf import Extrinsics    # TODO: extrinsics should be factored out of osf
from ouster.sdk._bindings.osf import LidarScanStreamMeta
from ouster.sdk._bindings.osf import LidarScanStream
from ouster.sdk._bindings.osf import StreamStats
from ouster.sdk._bindings.osf import StreamingInfo
from ouster.sdk._bindings.osf import Writer

from ouster.sdk._bindings.osf import slice_and_cast
from ouster.sdk._bindings.osf import dump_metadata
from ouster.sdk._bindings.osf import backup_osf_file_metablob
from ouster.sdk._bindings.osf import restore_osf_file_metablob
from ouster.sdk._bindings.osf import osf_file_modify_metadata

from .data import Scans
from .osf_scan_source import OsfScanSource
