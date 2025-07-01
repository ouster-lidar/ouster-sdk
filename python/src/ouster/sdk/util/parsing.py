"""R/W implementation of packet parsing.

Doesn't rely on custom C++ extensions (just numpy). Provides writable
view of packet data for testing and development.
"""
from typing import (List, Optional, Any, cast)

import numpy as np

import ouster.sdk.core as core
from ouster.sdk.core import (ChanField, UDPProfileLidar, LidarPacket)
from ouster.sdk._bindings.client import PacketWriter, get_field_types, FieldType
from ouster.sdk._bindings.client import scan_to_packets as _scan_to_packets


def default_scan_fields(
        profile: UDPProfileLidar,
        raw_headers: bool = False) -> List[core.FieldType]:
    """Get the default fields populated on scans for a profile.

    Convenient helper function if you want to tweak which fields are parsed
    into a LidarScan without listing out the defaults yourself.

    Args:
        profile: The lidar profile
        raw_headers: Include RAW_HEADERS field

    Returns:
        A field configuration that can be passed to `client.Scans`. or None for
        custom added UDPProfileLidar
    """

    fields = get_field_types(profile)

    if raw_headers:
        # Getting the optimal field type for RAW_HEADERS is not possible with
        # this method thus using the biggest UINT32 for RAW_HEADERS.

        # Alternatively you can use `osf.resolve_field_types()` that chooses
        # the more optimal dtype for RAW_HEADERS field
        fields.append(FieldType(ChanField.RAW_HEADERS, np.uint32))

    # type ignored because mypy insists you cant sort with no arguments
    fields.sort()  # type: ignore
    return fields.copy()


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
                    info: core.SensorInfo) -> List[LidarPacket]:
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


def terminator_packet(info: core.SensorInfo,
                      last_packet: LidarPacket) -> LidarPacket:
    """Makes a next after the last lidar packet buffer that finishes LidarScan.

    Main mechanism is to set the next frame_id (``frame_id + 1``) in uint16
    format of the lidar packet with some arbitrary data (filled with ``0xfe``).

    Such a next after the last lidar packet is needed for the ScanBatcher to
    correctly finish the scan (i.e. zero out column fields that are not
    arrived which is critical if used in a way when LidarScan object is reused.)

    NOTE[pb]: in Python it's almost always the new LidarScan is created from
              scratch and used as a receiver of lidar packet in the batching
              implementation, thus finalization with zeros and a proper cut can
              be skipped, however it's a huge difference from C++ batching loop
              impl and it's important to keep things closer to C++ and also have
              a normal way to cut the very last LidarScan in a streams.

    Args:
        info: metadata of the current batcher that is in use
        last_buf: the last buffer that was passed to batcher.

    """

    # get frame_id using core.PacketFormat
    pf = core.PacketFormat.from_info(info)
    curr_fid = pf.frame_id(last_packet.buf)

    pw = PacketWriter.from_info(info)
    last_buf_view = np.frombuffer(last_packet.buf,
                                  dtype=np.uint8,
                                  count=pf.lidar_packet_size)

    # get frame_id using parsing.py PacketFormat and compare with core result
    assert pw.frame_id(last_buf_view) == curr_fid, "core.PacketFormat " \
        "and parsing.py PacketFormat should get the same frame_id value from buffer"

    # making a dummy data for the terminal lidar_packet
    tpacket = LidarPacket(pw.lidar_packet_size)

    # update the frame_id so it causes the LidarScan finishing routine
    # NOTE: frame_id is uint16 datatype so we need to properly wrap it on +1
    pw.set_frame_id(tpacket, (curr_fid + 1) % 0xffff)

    return tpacket


def packets_to_scan(
        lidar_packets: List[LidarPacket],
        info: core.SensorInfo,
        *,
        fields: Optional[List[core.FieldType]] = None) -> core.LidarScan:
    """Batch buffers that belongs to a single scan into a LidarScan object.

    Errors if lidar_packets buffers do not belong to a single LidarScan. Typically
    inconsistent measurement_ids or frame_ids in buffers is an error, as well
    as more buffers then a single LidarScan of a specified PacketFormat can take.
    """
    w = info.format.columns_per_frame
    h = info.format.pixels_per_column
    _fields = fields if fields is not None else default_scan_fields(
        info.format.udp_profile_lidar)
    ls = core.LidarScan(h, w, _fields)
    pf = core.PacketFormat.from_info(info)
    batch = core.ScanBatcher(w, pf)
    for idx, packet in enumerate(lidar_packets):
        assert not batch(packet, ls), "lidar_packets buffers should belong to a " \
            f"single LidarScan, but {idx} of {len(lidar_packets)} buffers already " \
            "cut a LidarScan"

    if lidar_packets:
        assert batch(terminator_packet(info, lidar_packets[-1]),
                     ls), "Terminator buffer should cause a cut of LidarScan"

    # if all expected lidar buffers constraints are satisfied we have a batched
    # lidar scan at the end
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
