"""Utilities for hashing and comparing lidardata."""
from dataclasses import dataclass
from hashlib import md5
import json
from typing import Any, BinaryIO, Dict, List, Tuple

import numpy as np

from . import _bufstream as bufstream
from . import _sensor as sensor
from . import LidarPacket, LidarScan, ChanField, ColHeader
from . import batch_to_scan


def _md5(a: np.ndarray) -> str:
    """Get md5 hash of a numpy array."""
    return md5(np.ascontiguousarray(a)).hexdigest()


class ScanDigest:
    """Hashes of lidar data fields used for comparison in testing.

    Stores a hash of data from each channel and header field. Used to compare
    the results of parsing against known good outputs.
    """

    hashes: Dict[str, str]

    def __init__(self, **args: str):
        self.hashes = args

    def __eq__(self, other: Any):
        return self.hashes == other.hashes

    @classmethod
    def from_packet(cls, p: LidarPacket) -> 'ScanDigest':
        hashes = {}
        hashes.update({c.name: _md5(p.view(c)) for c in ChanField})
        hashes.update({h.name: _md5(p.view(h)) for h in ColHeader})

        return cls(**hashes)

    @classmethod
    def from_scan(cls, ls: LidarScan) -> 'ScanDigest':
        return cls(RANGE=_md5(ls.data[:, 0]),
                   REFLECTIVITY=_md5(ls.data[:, 3]),
                   INTENSITY=_md5(ls.data[:, 1]),
                   AMBIENT=_md5(ls.data[:, 2]),
                   TIMESTAMP=_md5(ls.ts))


def _digest_io(si: sensor.SensorInfo,
               b: BinaryIO) -> Tuple[List[ScanDigest], List[ScanDigest]]:
    pf = sensor.get_format(si)
    batch = batch_to_scan(si)
    packets, scans = [], []
    for buf in bufstream.read(b):
        packets.append(ScanDigest.from_packet(LidarPacket(bytearray(buf), pf)))
        ls = batch(buf)
        if ls is not None:
            scans.append(ScanDigest.from_scan(ls))
    return packets, scans


@dataclass
class StreamDigest:
    """Hashes and metadata for sensor udp data used for testing.

    Stores enough information to read channel data from packets stored on disk
    and compare their hashes against known good values.

    Attributes:
        file: Path to file containing a bufstream of packet data to check.
        meta: Sensor metadata used to parse packets into channel data.
        packets: List of known good hashes of channel data for each packet.
    """
    file: str
    meta: sensor.SensorInfo
    packets: List[ScanDigest]
    scans: List[ScanDigest]

    def check(self) -> None:
        """Check that data parsed from a packets on disk matches hashes."""
        with open(self.file, 'rb') as i:
            packets, scans = _digest_io(self.meta, i)
            assert self.packets == packets
            assert self.scans == scans

    def to_json(self) -> str:
        """Serialize to json."""
        return json.dumps(
            {
                'file': self.file,
                'meta': json.loads(str(self.meta)),
                'packets': [d.hashes for d in self.packets],
                'scans': [d.hashes for d in self.scans]
            },
            indent=4)

    @classmethod
    def from_bufstream(cls, file: str, meta: sensor.SensorInfo,
                       bufstream: BinaryIO) -> 'StreamDigest':
        """Generate a digest from a bufstream of packets."""
        packets, scans = _digest_io(meta, bufstream)
        return cls(file, meta, packets, scans)

    @classmethod
    def from_json(cls, s: str) -> 'StreamDigest':
        """Instantiate from json representation."""
        d = json.loads(s)
        return cls(
            file=d['file'],
            meta=sensor.parse_metadata(json.dumps(d['meta'])),
            packets=[ScanDigest(**hashes) for hashes in d.get('packets', [])],
            scans=[ScanDigest(**hashes) for hashes in d.get('scans', [])])
