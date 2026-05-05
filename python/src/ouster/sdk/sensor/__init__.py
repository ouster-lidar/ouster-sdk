"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

This module provides interfaces and utilities for interacting with Ouster lidar sensors directly.
It includes classes for streaming packets and scans from a sensor, configuring sensor settings, handling HTTP communication, and managing sensor-related errors.
"""
# flake8: noqa: F401 (unused imports)

from ouster.sdk._bindings.client import SensorPacketSource
from ouster.sdk._bindings.client import SensorScanSource

from ouster.sdk._bindings.client import get_config
from ouster.sdk._bindings.client import set_config
from ouster.sdk._bindings.client import SensorHttp
from ouster.sdk._bindings.client import Sensor as _Sensor
from ouster.sdk._bindings.client import ClientTimeout
from ouster.sdk._bindings.client import ClientError
from ouster.sdk._bindings.client import ClientOverflow

from ouster.sdk._bindings.client import MIN_VERSION

from ouster.sdk._bindings.client import set_http_api_headers
from ouster.sdk._bindings.client import set_http_api_prefix

# Import deprecation utilities for deprecated C++ symbols
from ouster.sdk._deprecation import deprecated_alias

deprecated_alias("min_version", "MIN_VERSION", MIN_VERSION, globals(), "0.16.0")
