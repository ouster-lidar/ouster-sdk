"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

This module contains wrappers for packet sources, frame borders, and pose validation.
Defines how to read packets, manage frames, and check pose validity.
"""
from typing import (Iterable, Iterator, List, Callable, Tuple, Union)
import logging
import numpy as np

from ouster.sdk._bindings.client import (SensorInfo, PacketFormat, LidarScan,
                      LidarPacket, ImuPacket, ZonePacket, Packet, PacketSource)

logger = logging.getLogger("ouster.sdk.core.core")


class Packets(PacketSource):
    """Create a :class:`PacketSource` from an existing iterator."""

    _it: Iterable[Union[LidarPacket, ImuPacket, ZonePacket]]
    _metadata: List[SensorInfo]

    def __init__(self, it: Iterable[Union[LidarPacket, ImuPacket, ZonePacket]], metadata: SensorInfo):
        """
        Args:
            it: A stream of packets
            metadata: Metadata for the packet stream
        """
        PacketSource.__init__(self)
        self._it = it
        self._metadata = [metadata]

    @property
    def sensor_info(self) -> List[SensorInfo]:
        return self._metadata

    def __iter__(self) -> Iterator[Tuple[int, Union[LidarPacket, ImuPacket, ZonePacket]]]:
        """Return the underlying iterator."""
        for packet in self._it:
            yield (0, packet)

    def close(self) -> None:
        pass

    @property
    def is_live(self) -> bool:
        return False


class FrameBorder:
    """Create callable helper that indicates the cross frames packets."""

    def __init__(self, meta: SensorInfo, pred: Callable[[Packet], bool] = lambda _: True):
        self._last_f_id = -1
        self._last_packet_ts = None
        self._last_packet_res = False
        self._pred = pred
        self._pf = PacketFormat(meta)

    def __call__(self, packet: Packet) -> bool:
        if isinstance(packet, LidarPacket):
            # don't examine packets again
            if (self._last_packet_ts and (packet.host_timestamp != 0) and
                    self._last_packet_ts == packet.host_timestamp):
                return self._last_packet_res
            f_id = self._pf.frame_id(packet.buf)
            changed = (self._last_f_id != -1 and f_id != self._last_f_id)
            self._last_packet_res = changed and self._pred(packet)
            self._last_f_id = f_id
            return self._last_packet_res
        return False


def first_valid_column_pose(scan: LidarScan) -> np.ndarray:
    """Return first valid column pose of a LidarScan"""
    return scan.pose[scan.get_first_valid_column()]


def last_valid_column_pose(scan: LidarScan) -> np.ndarray:
    """Return last valid column pose of a LidarScan"""
    return scan.pose[scan.get_last_valid_column()]


def valid_packet_idxs(scan: LidarScan) -> np.ndarray:
    """Checks for valid packets that was used in in the scan construction"""
    valid_cols = scan.status & 0x1
    valid_packet_ts = scan.packet_timestamp != 0
    sp = np.split(valid_cols, scan.packet_timestamp.shape[0])
    # here we consider the packet is valid when either one is true:
    #   - any columns in the packet has a valid status
    #   - packet_timestamp is not zero, which may occur even when
    #     all columns/px data in invalid state within the packet.
    #     It means that we received the packet without per px data
    #     but with all other headers in place
    valid_packets = np.logical_or(np.any(sp, axis=1), valid_packet_ts)
    return np.nonzero(valid_packets)[0]


def poses_present(scan: LidarScan) -> bool:
    """Check whether any of scan.pose in not identity"""
    return not np.allclose(np.eye(4), scan.pose)
