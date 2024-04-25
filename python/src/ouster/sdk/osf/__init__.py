"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

API to work with OSF files
"""
# flake8: noqa (unused imports)

from ._osf import Reader
from ._osf import MessageRef
from ._osf import ChunkRef
from ._osf import MetadataStore
from ._osf import MetadataEntry
from ._osf import LidarSensor
from ._osf import Extrinsics    # TODO: extrinsics should be factored out of osf
from ._osf import LidarScanStreamMeta
from ._osf import LidarScanStream
from ._osf import StreamStats
from ._osf import StreamingInfo
from ._osf import ChunksLayout
from ._osf import Writer

from ._osf import slice_and_cast
from ._osf import dump_metadata
from ._osf import backup_osf_file_metablob
from ._osf import restore_osf_file_metablob
from ._osf import osf_file_modify_metadata

from .data import Scans
from .osf_scan_source import OsfScanSource
