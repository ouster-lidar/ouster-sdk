"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

Ouster Bag file support.
Provides classes and utilities for reading, decoding, and processing Ouster lidar sensor data from ROS bag files.
Includes tools for accessing raw sensor packets and assembling complete lidar frames.
"""
# flake8: noqa: F401 (unused imports)

from .bag_packet_source import BagPacketSource
from .bag_frame_set_source import BagFrameSetSource

from ouster.sdk._deprecation import deprecated_alias
deprecated_alias("BagScanSource", "BagFrameSetSource", BagFrameSetSource, globals(), "1.0")
