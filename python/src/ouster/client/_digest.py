"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Utilities for hashing and comparing lidardata.
"""
from collections import defaultdict
from dataclasses import dataclass
import hashlib
import json
from typing import (Dict, List, Iterable)

import numpy as np

from .data import (LidarPacket, LidarScan, ColHeader)
from .core import (Packets, PacketSource, Scans)


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
        for k, v in self.hashes.items():
            assert other.hashes.get(k) == v, f"Match failure key: {k}"

    @classmethod
    def from_packet(cls, packet: LidarPacket) -> 'FieldDigest':
        return cls.from_packets([packet])

    @classmethod
    def from_packets(cls, packets: Iterable[LidarPacket]) -> 'FieldDigest':
        # hashlib._Hash doesn't exist at runtime
        hashes: Dict[str, 'hashlib._Hash'] = defaultdict(hashlib.md5)

        for packet in packets:
            # TODO: add packet headers
            for h in ColHeader:
                hashes[h.name].update(packet.header(h).tobytes())
            for f in packet.fields:
                hashes[f.name].update(packet.field(f).tobytes())

        return cls(**{k: v.hexdigest() for k, v in hashes.items()})

    @classmethod
    def from_scan(cls, ls: LidarScan) -> 'FieldDigest':
        hashes = {}

        hashes['FRAME_ID'] = str(ls.frame_id)

        # TODO: remove astype
        hashes['TIMESTAMP'] = _md5(ls.timestamp.astype(np.uint64))
        hashes['STATUS'] = _md5(ls.status.astype(np.uint64))
        hashes['MEASUREMENT_ID'] = _md5(ls.measurement_id.astype(np.uint16))

        # deprecated, zeroed out for non-legacy UDP profiles
        hashes['ENCODER_COUNT'] = _md5(
            ls.header(ColHeader.ENCODER_COUNT).astype(np.uint64))

        hashes.update({c.name: _md5(ls.field(c)) for c in ls.fields})

        return cls(**hashes)


@dataclass
class StreamDigest:
    """Hashes and metadata for sensor udp data used for testing.

    Stores enough information to read channel data from packets stored on disk
    and compare their hashes against known good values.

    Attributes:
        packet_hash: Hashed fields of the UDP packets
        scans: List of hashed fields of the LidarScan
    """
    packet_hash: FieldDigest
    scans: List[FieldDigest]

    def check(self, other: 'StreamDigest'):
        """Check that this digest is compatible with another.

        Assert that for each packet and scan recorded in this digest, all
        hashes match hashes in the other digest. The other digest may have
        additional hashes for fields that aren't present here.
        """

        assert len(self.scans) == len(other.scans)
        self.packet_hash.check(other.packet_hash)

        for s, t in zip(self.scans, other.scans):
            s.check(t)

    def to_json(self) -> str:
        """Serialize to json."""
        return json.dumps(
            {
                'packet_hash': self.packet_hash.hashes,
                'scans': [d.hashes for d in self.scans]
            },
            indent=4)

    @classmethod
    def from_packets(cls, source: PacketSource) -> 'StreamDigest':
        """Generate a digest from a packet stream."""

        plist = [p for p in source if isinstance(p, LidarPacket)]
        packets = Packets(plist, source.metadata)

        scan_digests = list(map(FieldDigest.from_scan, Scans(packets)))
        packet_digest = FieldDigest.from_packets(plist)

        return cls(packet_hash=packet_digest, scans=scan_digests)

    @classmethod
    def from_json(cls, json_data: str) -> 'StreamDigest':
        """Instantiate from json representation."""
        d = json.loads(json_data)

        return cls(
            packet_hash=FieldDigest(**d['packet_hash']),
            scans=[FieldDigest(**hashes) for hashes in d.get('scans', [])])
