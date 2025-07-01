"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Python sensor client
"""
# flake8: noqa (unused imports)
import warnings

warnings.warn("Namespace ouster.sdk.client is deprecated: use ouster.sdk.core and ouster.sdk.sensor instead. "
              "Namespace ouster.sdk.client will be removed in the upcoming release.",
              DeprecationWarning, stacklevel=2)

from ouster.sdk.core import Severity
from ouster.sdk.core import SensorInfo
from ouster.sdk.core import ProductInfo
from ouster.sdk.core import DataFormat
from ouster.sdk.core import LidarMode
from ouster.sdk.core import TimestampMode
from ouster.sdk.core import OperatingMode
from ouster.sdk.core import MultipurposeIOMode
from ouster.sdk.core import Polarity
from ouster.sdk.core import NMEABaudRate
from ouster.sdk.core import UDPProfileLidar
from ouster.sdk.core import UDPProfileIMU
from ouster.sdk.core import SensorConfig
from ouster.sdk.core import SensorCalibration
from ouster.sdk.core import SensorHttp
from ouster.sdk.core import ShotLimitingStatus
from ouster.sdk.core import ThermalShutdownStatus
from ouster.sdk.core import FieldClass
from ouster.sdk.core import FullScaleRange
from ouster.sdk.core import ReturnOrder
from ouster.sdk.core import init_logger
from ouster.sdk.core import FieldType
from ouster.sdk.core import LidarScan
from ouster.sdk.core import get_field_types
from ouster.sdk.core import Packet
from ouster.sdk.core import LidarPacket
from ouster.sdk.core import ImuPacket
from ouster.sdk.core import PacketValidationFailure
from ouster.sdk.core import PacketFormat
from ouster.sdk.core import PacketWriter
from ouster.sdk.core import Version
from ouster.sdk.core import parse_and_validate_metadata
from ouster.sdk.core import parse_and_validate_sensor_config
from ouster.sdk.core import ValidatorIssues
from ouster.sdk.core import ValidatorEntry
from ouster.sdk.core import ScanBatcher
from ouster.sdk.core import ScanSource
from ouster.sdk.core import dewarp
from ouster.sdk.core import transform
from ouster.sdk.core import LONG_HTTP_REQUEST_TIMEOUT_SECONDS, SHORT_HTTP_REQUEST_TIMEOUT_SECONDS

from ouster.sdk.core.data import BufferT
from ouster.sdk.core.data import ColHeader
from ouster.sdk.core.data import XYZLut
from ouster.sdk.core.data import XYZLutFloat
from ouster.sdk.core.data import destagger
from ouster.sdk.core.data import stagger
from ouster.sdk.core.data import packet_ts
from ouster.sdk.core.data import ChanField

from ouster.sdk.core.reduced_scan_source import ReducedScanSource as MultiReducedScanSource
from ouster.sdk.core.masked_scan_source import MaskedScanSource as MultiMaskedScanSource
from ouster.sdk.core.clipped_scan_source import ClippedScanSource as MultiClippedScanSource

from ouster.sdk.core.core import PacketSource
from ouster.sdk.core.core import Packets
from ouster.sdk.core.core import FrameBorder
from ouster.sdk.core.core import first_valid_column
from ouster.sdk.core.core import last_valid_column
from ouster.sdk.core.core import first_valid_column_ts
from ouster.sdk.core.core import first_valid_packet_ts
from ouster.sdk.core.core import last_valid_column_ts
from ouster.sdk.core.core import last_valid_packet_ts
from ouster.sdk.core.core import first_valid_column_pose
from ouster.sdk.core.core import last_valid_column_pose
from ouster.sdk.core.core import valid_packet_idxs
from ouster.sdk.core.core import poses_present

from ouster.sdk.core.multi import Scans

from ouster.sdk.sensor import get_config
from ouster.sdk.sensor import set_config
from ouster.sdk.sensor import _Sensor
from ouster.sdk.sensor import ClientError
from ouster.sdk.sensor import ClientTimeout
from ouster.sdk.sensor import ClientOverflow
from ouster.sdk.sensor import SensorPacketSource
from ouster.sdk.sensor import SensorScanSource

from ouster.sdk.core import _utils
from ouster.sdk.core import core
from ouster.sdk.core import data
from ouster.sdk.core import io_types
from ouster.sdk.core import multi
from ouster.sdk.core import scan_ops
