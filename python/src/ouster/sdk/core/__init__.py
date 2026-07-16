"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Core namespace provides core classes, functions, and utilities for working with Ouster lidar sensor data in Python.
It covers packet and frame handling, data transformations, and various helpers for processing and managing lidar data.
"""
# flake8: noqa (unused imports)

import importlib

from ouster.sdk._kwarg_aliases import install_client_kwarg_aliases

install_client_kwarg_aliases(importlib.import_module("ouster.sdk._bindings.client"))

from ouster.sdk._bindings.client import Severity
from ouster.sdk._bindings.client import SensorInfo
from ouster.sdk._bindings.client import ProductInfo
from ouster.sdk._bindings.client import DataFormat
from ouster.sdk._bindings.client import LidarMode
from ouster.sdk._bindings.client import TimestampMode
from ouster.sdk._bindings.client import OperatingMode
from ouster.sdk._bindings.client import MultipurposeIOMode
from ouster.sdk._bindings.client import Polarity
from ouster.sdk._bindings.client import NMEABaudRate
from ouster.sdk._bindings.client import UDPProfileLidar
from ouster.sdk._bindings.client import UDPProfileIMU
from ouster.sdk._bindings.client import HeaderType
from ouster.sdk._bindings.client import SensorConfig
from ouster.sdk._bindings.client import CalibrationStatus
from ouster.sdk._bindings.client import ShotLimitingStatus
from ouster.sdk._bindings.client import ThermalShutdownStatus
from ouster.sdk._bindings.client import FieldClass
from ouster.sdk._bindings.client import FieldDecodeInfo
from ouster.sdk._bindings.client import FullScaleRange
from ouster.sdk._bindings.client import ReturnOrder
from ouster.sdk._bindings.client import init_logger
from ouster.sdk._bindings.client import FieldType
from ouster.sdk._bindings.client import LidarFrame
from ouster.sdk._bindings.client import get_field_types
from ouster.sdk._bindings.client import Packet
from ouster.sdk._bindings.client import LidarPacket
from ouster.sdk._bindings.client import ImuPacket
from ouster.sdk._bindings.client import ZonePacket
from ouster.sdk._bindings.client import ZoneSet
from ouster.sdk._bindings.client import Zone
from ouster.sdk._bindings.client import ZoneMode
from ouster.sdk._bindings.client import ZoneState
from ouster.sdk._bindings.client import Stl
from ouster.sdk._bindings.client import Zrb
from ouster.sdk._bindings.client import CoordinateFrame
from ouster.sdk._bindings.client import ZoneSetOutputFilter
from ouster.sdk._bindings.client import PacketValidationFailure
from ouster.sdk._bindings.client import PacketFormat
from ouster.sdk._bindings.client import Version
from ouster.sdk._bindings.client import parse_and_validate_metadata
from ouster.sdk._bindings.client import parse_and_validate_sensor_config
from ouster.sdk._bindings.client import ValidatorIssues
from ouster.sdk._bindings.client import ValidatorEntry
from ouster.sdk._bindings.client import FrameBatcher
from ouster.sdk._bindings.client import FrameSetSource
from ouster.sdk._bindings.client import FrameSetSourceMetadataSet
from ouster.sdk._bindings.client import ClassMap, ClassMapSet
from ouster.sdk._bindings.client import dewarp
from ouster.sdk._bindings.client import transform
from ouster.sdk._bindings.client import interp_pose
from ouster.sdk._bindings.client import interp_pose_float
from ouster.sdk._bindings.client import Pose
from ouster.sdk._bindings.client import euler_pose_to_matrix
from ouster.sdk._bindings.client import matrix_to_euler
from ouster.sdk._bindings.client import quaternion_pose_to_matrix
from ouster.sdk._bindings.client import get_rot_matrix_to_align_to_gravity
from ouster.sdk._bindings.client import LONG_HTTP_REQUEST_TIMEOUT_SECONDS, SHORT_HTTP_REQUEST_TIMEOUT_SECONDS
from ouster.sdk._bindings.client import collate
from ouster.sdk._bindings.client import read_pointcloud
from ouster.sdk._bindings.client import voxel_downsample_3d
from ouster.sdk._bindings.client import voxel_downsample_xd
from ouster.sdk._bindings.client import voxel_downsample_xd as voxel_downsample
from ouster.sdk._bindings.client import VoxelDownsampleStrategy
from ouster.sdk._bindings.client import add_custom_profile
from ouster.sdk._bindings.client import MultiFrameSetSource
from ouster.sdk._bindings.client import FrameSet
from ouster.sdk._bindings.client import BloomReductionOptimization
from ouster.sdk._bindings.client import INVALID_VERSION
from ouster.sdk._bindings.client import PacketSource
from ouster.sdk._bindings.client import XYZLut
from ouster.sdk._bindings.client import XYZLutFloat
from ouster.sdk._bindings.client import AutoExposure
from ouster.sdk._bindings.client import BeamUniformityCorrector
from ouster.sdk._bindings.client import LocalToneMapper
from ouster.sdk._bindings.client import frame_to_packets
from ouster.sdk._bindings.client import VoxelHashMap3d
from ouster.sdk._bindings.client import VoxelHashMapXd
from ouster.sdk._bindings.client import client_version
from ouster.sdk._bindings.client import Object
from ouster.sdk._bindings.client import pose_at_timestamp
from ouster.sdk._bindings.client import restore_instance_ids
from ouster.sdk._bindings.client import destagger
from ouster.sdk._bindings.client import IoType
from ouster.sdk._bindings.client import IoType as OusterIoType
from ouster.sdk._bindings.client import extension_from_io_type
from ouster.sdk._bindings.client import io_type_from_extension
from ouster.sdk._bindings.client import io_type


from .data import BufferT
from .data import ColHeader
from .data import stagger
from .data import packet_ts
from .data import ChanField


from .reduced_frame_set_source import ReducedFrameSetSource
from .masked_frame_set_source import MaskedFrameSetSource
from .clipped_frame_set_source import ClippedFrameSetSource
from .selected_frame_set_source import SelectedFrameSetSource


from .core import Packets
from .core import FrameBorder
from .core import first_valid_column_pose
from .core import last_valid_column_pose

# Zone monitor classes and constants
from .zone_common import EmulatedZoneMon, MAX_ACTIVE_ZONES  # noqa: F401

ZONE_STATES_FIELDNAME = 'ZONE_STATES'
ZONE_OCCUPANCY_FIELDNAME = 'ZONE_MASK'

from ouster.sdk._deprecation import deprecated_alias, wrap_deprecated_kwargs, warn_deprecated, _get_deprecation_message
frame_to_packets = wrap_deprecated_kwargs(frame_to_packets, lidar_scan='lidar_frame')

def scan_to_packets(*args, **kwargs):
    warn_deprecated(
        _get_deprecation_message("scan_to_packets", "frame_to_packets", "1.0"),
        stacklevel=2,
    )
    return frame_to_packets(*args, **kwargs)

deprecated_alias("ScanBatcher", "FrameBatcher", FrameBatcher, globals(), "1.0")
deprecated_alias("LidarScanSet", "FrameSet", FrameSet, globals(), "1.0")
deprecated_alias("LidarScan", "LidarFrame", LidarFrame, globals(), "1.0")
deprecated_alias("ScanSource", "FrameSetSource", FrameSetSource, globals(), "1.0")
deprecated_alias("ScanSourceMetadataSet", "FrameSetSourceMetadataSet", FrameSetSourceMetadataSet, globals(), "1.0")
deprecated_alias("MultiScanSource", "MultiFrameSetSource", MultiFrameSetSource, globals(), "1.0")
deprecated_alias("ClippedScanSource", "ClippedFrameSetSource", ClippedFrameSetSource, globals(), "1.0")
deprecated_alias("MaskedScanSource", "MaskedFrameSetSource", MaskedFrameSetSource, globals(), "1.0")
deprecated_alias("ReducedScanSource", "ReducedFrameSetSource", ReducedFrameSetSource, globals(), "1.0")

from ouster.sdk._kwarg_aliases import install_core_python_kwarg_aliases
install_core_python_kwarg_aliases()
