"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Utilities for hashing and comparing lidar data.
"""
from collections import defaultdict
from dataclasses import dataclass
import hashlib
import json
import logging
from typing import (Dict, List, Iterable)
from itertools import tee

import numpy as np

from ouster.sdk.core.data import ColHeader
from ouster.sdk.core import (LidarFrame, LidarPacket, PacketSource, PacketFormat)
from ouster.sdk._bindings.client import FrameBatcher


def _md5(a: np.ndarray) -> str:
    """Get md5 hash of a numpy array."""
    return hashlib.md5(a.tobytes()).hexdigest()


class FieldDigest:
    """Hashes of lidar data fields used for comparison in testing.

    Stores a hash of data from each channel and header field. Used to compare
    the results of parsing against known good outputs.
    """

    hashes: Dict[str, str]

    def __init__(self, **args: str):
        self.hashes = args

    def __eq__(self, other: object):
        if isinstance(other, FieldDigest):
            return self.hashes == other.hashes
        return False

    def check(self, other: 'FieldDigest'):
        for k, v in sorted(self.hashes.items()):
            # TODO - remove when remove deprecated 'ENCODER_COUNT'
            # TODO (cnt)  from packet parsing and from digest test data
            if k != 'ENCODER_COUNT':
                assert other.hashes.get(k) == v, f"Match failure key: {k}"

    @classmethod
    def from_packet(cls, packet: LidarPacket, pf: PacketFormat) -> 'FieldDigest':
        return cls.from_packets([packet], pf)

    @classmethod
    def from_packets(cls, packets: Iterable[LidarPacket], pf: PacketFormat) -> 'FieldDigest':
        # hashlib._Hash doesn't exist at runtime
        hashes: Dict[str, 'hashlib._Hash'] = defaultdict(hashlib.md5)

        for idx, packet in enumerate(packets):
            # TODO: add packet headers
            for h in ColHeader:
                hashes[h.name].update(pf.packet_header(h, packet.buf).tobytes())
            for field_name in pf.fields:
                hashes[field_name].update(pf.packet_field(field_name, packet.buf).tobytes())

        return cls(**{k: v.hexdigest() for k, v in hashes.items()})

    @classmethod
    def from_frame(cls, lf: LidarFrame) -> 'FieldDigest':
        hashes = {}

        hashes['FRAME_ID'] = str(lf.frame_id)

        # TODO: remove astype
        hashes['TIMESTAMP'] = _md5(lf.timestamp.astype(np.uint64))
        hashes['STATUS'] = _md5(lf.status.astype(np.uint64))
        hashes['MEASUREMENT_ID'] = _md5(lf.measurement_id.astype(np.uint16))

        hashes.update({field_name: _md5(lf.field(field_name)) for field_name in lf.fields})

        return cls(**hashes)


@dataclass
class StreamDigest:
    """Hashes and metadata for sensor udp data used for testing.

    Stores enough information to read channel data from packets stored on disk
    and compare their hashes against known good values.

    Attributes:
        packet_hash: Hashed fields of the UDP packets
        frames: List of hashed fields of the LidarFrame
    """
    packet_hash: FieldDigest
    frames: List[FieldDigest]

    def check(self, other: 'StreamDigest'):
        """Check that this digest is compatible with another.

        Assert that for each packet and frame recorded in this digest, all
        hashes match hashes in the other digest. The other digest may have
        additional hashes for fields that aren't present here.
        """

        assert len(self.frames) == len(other.frames)
        logging.debug("Checking packet hash..")
        self.packet_hash.check(other.packet_hash)

        logging.debug("Checking frame hash..")
        for s, t in zip(self.frames, other.frames):
            s.check(t)

    def to_json(self) -> str:
        """Serialize to json."""
        return json.dumps(
            {
                'packet_hash': self.packet_hash.hashes,
                'frames': [d.hashes for d in self.frames]
            },
            indent=4)

    @classmethod
    def from_packets(cls, source: PacketSource) -> 'StreamDigest':
        """Generate a digest from a packet stream."""

        source1, source2, = tee(source)

        allpackets = [p for idx, p in source2]
        plist = [p for idx, p in source1 if isinstance(p, LidarPacket)]
        logging.debug(f"Creating digest with {len(plist)} Lidar packets out of total {len(allpackets)}")
        metadata = source.sensor_info[0]
        pf = PacketFormat(metadata)
        batcher = FrameBatcher(metadata)
        frame = LidarFrame(metadata)

        def batch():
            nonlocal frame
            new_frame = True
            for p in plist:
                new_frame = False
                if batcher.batch(p, frame):
                    print("frame finished")
                    yield frame
                    frame = LidarFrame(metadata)
                    new_frame = True
            if not new_frame:
                yield frame

        frames_list1, frames_list2 = tee(batch())
        for frame in frames_list1:
            logging.debug(f"Packets for StreamDigest created a frame with complete status {frame.complete()}")

        # frame_digests = list(map(FieldDigest.from_frame, Frames(packets)))
        frame_digests = list(map(FieldDigest.from_frame, frames_list2))
        packet_digest = FieldDigest.from_packets(plist, pf)

        return cls(packet_hash=packet_digest, frames=frame_digests)

    @classmethod
    def from_json(cls, json_data: str) -> 'StreamDigest':
        """Instantiate from json representation."""
        d = json.loads(json_data)

        frames_data = d.get('frames', d.get('scans', []))
        return cls(
            packet_hash=FieldDigest(**d['packet_hash']),
            frames=[FieldDigest(**hashes) for hashes in frames_data])
