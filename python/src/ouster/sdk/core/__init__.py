"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Core namespace provides core classes, functions, and utilities for working with Ouster lidar sensor data in Python.
It covers packet and scan handling, data transformations, and various helpers for processing and managing lidar data.
"""
# flake8: noqa (unused imports)

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
from ouster.sdk._bindings.client import SensorHttp
from ouster.sdk._bindings.client import ShotLimitingStatus
from ouster.sdk._bindings.client import ThermalShutdownStatus
from ouster.sdk._bindings.client import FieldClass
from ouster.sdk._bindings.client import FullScaleRange
from ouster.sdk._bindings.client import ReturnOrder
from ouster.sdk._bindings.client import init_logger
from ouster.sdk._bindings.client import FieldType
from ouster.sdk._bindings.client import LidarScan
from ouster.sdk._bindings.client import get_field_types
from ouster.sdk._bindings.client import Packet
from ouster.sdk._bindings.client import LidarPacket
from ouster.sdk._bindings.client import ImuPacket
from ouster.sdk._bindings.client import ZonePacket
from ouster.sdk._bindings.client import PacketValidationFailure
from ouster.sdk._bindings.client import PacketFormat
from ouster.sdk._bindings.client import PacketWriter
from ouster.sdk._bindings.client import Version
from ouster.sdk._bindings.client import parse_and_validate_metadata
from ouster.sdk._bindings.client import parse_and_validate_sensor_config
from ouster.sdk._bindings.client import ValidatorIssues
from ouster.sdk._bindings.client import ValidatorEntry
from ouster.sdk._bindings.client import ScanBatcher
from ouster.sdk._bindings.client import ScanSource
from ouster.sdk._bindings.client import dewarp
from ouster.sdk._bindings.client import transform
from ouster.sdk._bindings.client import interp_pose
from ouster.sdk._bindings.client import interp_pose_float
from ouster.sdk._bindings.client import euler_pose_to_matrix
from ouster.sdk._bindings.client import quaternion_pose_to_matrix
from ouster.sdk._bindings.client import LONG_HTTP_REQUEST_TIMEOUT_SECONDS, SHORT_HTTP_REQUEST_TIMEOUT_SECONDS
from ouster.sdk._bindings.client import collate
from ouster.sdk._bindings.client import read_pointcloud
from ouster.sdk._bindings.client import voxel_downsample
from ouster.sdk._bindings.client import add_custom_profile
from ouster.sdk._bindings.client import MultiScanSource
from ouster.sdk._bindings.client import LidarScanSet
from ouster.sdk._bindings.client import BloomReductionOptimization
from ouster.sdk._bindings.client import INVALID_VERSION
from ouster.sdk._bindings.client import normals

from .data import BufferT
from .data import ColHeader
from .data import XYZLut
from .data import XYZLutFloat
from .data import destagger
from .data import stagger
from .data import packet_ts
from .data import ChanField

from .reduced_scan_source import ReducedScanSource
from .masked_scan_source import MaskedScanSource
from .clipped_scan_source import ClippedScanSource

from .io_types import OusterIoType
from .io_types import extension_from_io_type
from .io_types import io_type_from_extension
from .io_types import io_type

from .core import PacketSource
from .core import Packets
from .core import FrameBorder
from .core import first_valid_column_pose
from .core import last_valid_column_pose
from .core import valid_packet_idxs
from .core import poses_present

from .multi import Scans           # type: ignore

# Import deprecation utilities
from ouster.sdk._deprecation import deprecated_alias

# Add deprecated aliases based on C++ deprecations
deprecated_alias("version", "Version", Version, globals(), "0.16.0")
deprecated_alias("lidar_mode", "LidarMode", LidarMode, globals(), "0.16.0")
deprecated_alias("timestamp_mode", "TimestampMode", TimestampMode, globals(), "0.16.0")
deprecated_alias("sensor_config", "SensorConfig", SensorConfig, globals(), "0.16.0")
deprecated_alias("data_format", "DataFormat", DataFormat, globals(), "0.16.0")
deprecated_alias("product_info", "ProductInfo", ProductInfo, globals(), "0.16.0")
deprecated_alias("sensor_info", "SensorInfo", SensorInfo, globals(), "0.16.0")
deprecated_alias("packet_format", "PacketFormat", PacketFormat, globals(), "0.16.0")
deprecated_alias("SensorCalibration", "CalibrationStatus", CalibrationStatus, globals(), "0.16.0")
deprecated_alias("invalid_version", "INVALID_VERSION", INVALID_VERSION, globals(), "0.16.0")
