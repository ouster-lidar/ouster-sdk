"""R/W implementation of packet parsing.

Doesn't rely on custom C++ extensions (just numpy). Provides writable
view of packet data for testing and development.
"""
from typing import (List, Optional, Any, cast)

import numpy as np

import ouster.sdk.core as core
from ouster.sdk.core import Packet
from ouster.sdk._bindings.client import PacketWriter
from ouster.sdk._bindings.client import scan_to_packets as _scan_to_packets


def tohex(data: core.BufferT) -> str:
    """Makes a hex string for debug print outs of buffers.

    Selects the biggest devisor of np.uint32, np.uint16 or np.uint8 for making
    a hex output of the provided data. (clunky but usefull for debugging)

    """
    if len(data):
        if isinstance(data, np.ndarray) and not data.flags['C_CONTIGUOUS']:
            data_cont = np.ascontiguousarray(data)
        else:
            data_cont = cast(np.ndarray[Any, Any], data)
        # selecting the biggest dtype that devides num bytes exactly, because
        # vectorized hex can't work with data if it's not a multiple of element
        # type
        bytes_len = np.frombuffer(data_cont, dtype=np.uint8).size
        dtype = {
            0: np.uint32,
            1: np.uint8,
            2: np.uint16,
            3: np.uint8
        }[bytes_len % 4]
        return np.vectorize(hex)(np.frombuffer(data_cont, dtype=dtype))
    else:
        return "[]"


def scan_to_packets(ls: core.LidarScan,
                    info: core.SensorInfo) -> List[Packet]:
    """Converts LidarScan to a lidar_packet buffers

    Args:
        ls: LidarScan; if LidarScan has RAW_HEADERS field, packet headers
            are recreated to how they were in the original packets
        info: metadata of the `ls` scan

    Returns:
        A set of lidar packets that will produce the same LidarScan if passed
        through the ScanBatcher again (less fields data)
    """
    return _scan_to_packets(ls, PacketWriter.from_info(info), info.init_id, int(info.sn))


def packets_to_scan(
        packets: List[Packet],
        info: core.SensorInfo,
        *,
        fields: Optional[List[core.FieldType]] = None) -> core.LidarScan:
    """Batch buffers that belongs to a single scan into a LidarScan object."""
    if fields is None:
        ls = core.LidarScan(info)
    else:
        ls = core.LidarScan(info, fields)
    batch = core.ScanBatcher(info)
    for packet in packets:
        batch(packet, ls)
    # scan finalisation is not necessary as scan is freshly created here

    return ls


def cut_raw32_words(ls: core.LidarScan) -> core.LidarScan:
    cut_chans = [
        core.ChanField.RAW32_WORD1,
        core.ChanField.RAW32_WORD2,
        core.ChanField.RAW32_WORD3,
        core.ChanField.RAW32_WORD4,
        core.ChanField.RAW32_WORD5,
        core.ChanField.RAW32_WORD6,
        core.ChanField.RAW32_WORD7,
        core.ChanField.RAW32_WORD8,
        core.ChanField.RAW32_WORD9
    ]

    import ouster.sdk.osf as osf
    new_fields = {c: ls.field(c).dtype for c in ls.fields if c not in cut_chans}
    return osf.slice_and_cast(ls, new_fields)
