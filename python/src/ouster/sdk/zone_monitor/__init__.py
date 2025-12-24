"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.

This module provides classes and utilities for configuring, managing, and monitoring zones in Ouster lidar data.
"""
from ouster.sdk._bindings.client import ZoneSet, Zone, ZoneMode, Stl, Zrb, ZoneState, \
    CoordinateFrame, ZoneSetOutputFilter  # noqa: F401
from ouster.sdk.zone_monitor.zone_common import EmulatedZoneMon, MAX_ACTIVE_ZONES  # noqa: F401

ZONE_STATES_FIELDNAME = 'ZONE_STATES'
ZONE_OCCUPANCY_FIELDNAME = 'ZONE_MASK'
