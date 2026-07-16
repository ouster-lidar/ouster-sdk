"""R/W implementation of packet parsing.

Doesn't rely on custom C++ extensions (just numpy). Provides writable
view of packet data for testing and development.
"""
from typing import (List, Optional, Any, cast, Union)

import numpy as np

import ouster.sdk.core as core
from ouster.sdk.core import LidarPacket, ZonePacket, ImuPacket, PacketFormat
from ouster.sdk._bindings.client import frame_to_packets as _frame_to_packets


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


def frame_to_packets(lf: core.LidarFrame,
                    info: core.SensorInfo) -> List[Union[LidarPacket, ZonePacket, ImuPacket]]:
    """Converts LidarFrame to a lidar_packet buffers

    Args:
        lf: LidarFrame; if LidarFrame has RAW_HEADERS field, packet headers
            are recreated to how they were in the original packets
        info: metadata of the `lf` frame

    Returns:
        A set of lidar packets that will produce the same LidarFrame if passed
        through the FrameBatcher again (less fields data)
    """
    return _frame_to_packets(lf, PacketFormat.from_info(info), info.init_id, int(info.sn))


def packets_to_frame(
        packets: List[Union[LidarPacket, ZonePacket, ImuPacket]],
        info: core.SensorInfo,
        *,
        fields: Optional[List[core.FieldType]] = None) -> core.LidarFrame:
    """Batch buffers that belongs to a single frame into a LidarFrame object."""
    if fields is None:
        lf = core.LidarFrame(info)
    else:
        lf = core.LidarFrame(info, fields)
    batch = core.FrameBatcher(info)
    for packet in packets:
        batch(packet, lf)
    # frame finalisation is not necessary as frame is freshly created here

    return lf


def cut_raw32_words(lf: core.LidarFrame) -> core.LidarFrame:
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
    new_fields = {c: lf.field(c).dtype for c in lf.fields if c not in cut_chans}
    return osf.slice_and_cast(lf, new_fields)
