"""Utilities for hashing and comparing lidardata."""
from dataclasses import dataclass, asdict
from hashlib import md5
import json
from typing import BinaryIO, List

import numpy as np

from . import _bufstream as bufstream
from . import _sensor as sensor
from . import Packet, Channel, ColHeader


def _md5(a: np.ndarray) -> str:
    """Get md5 hash of a numpy array."""
    return md5(np.ascontiguousarray(a)).hexdigest()


@dataclass
class ScanDigest:
    """Hashes of lidar data fields used for comparison in testing.

    Stores a hash of data from each channel and header field. Used to compare
    the results of parsing against known good outputs.
    """

    ranges: str
    reflectivity: str
    intensity: str
    ambient: str

    timestamp: str
    encoder: str
    measurement_id: str
    frame_id: str
    valid: str

    @classmethod
    def from_packet(cls, p: Packet) -> 'ScanDigest':
        return cls(ranges=_md5(p.view(Channel.RANGE)),
                   reflectivity=_md5(p.view(Channel.REFLECTIVITY)),
                   intensity=_md5(p.view(Channel.INTENSITY)),
                   ambient=_md5(p.view(Channel.AMBIENT)),
                   timestamp=_md5(p.view(ColHeader.TIMESTAMP)),
                   encoder=_md5(p.view(ColHeader.ENCODER_COUNT)),
                   measurement_id=_md5(p.view(ColHeader.MEASUREMENT_ID)),
                   frame_id=_md5(p.view(ColHeader.FRAME_ID)),
                   valid=_md5(p.view(ColHeader.VALID)))


def _digest_io(pf: sensor.PacketFormat, b: BinaryIO) -> List[ScanDigest]:
    return [
        ScanDigest.from_packet(Packet(bytearray(buf), pf))
        for buf in bufstream.read(b)
    ]


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

    def check(self) -> None:
        """Check that data parsed from a packets on disk matches hashes."""
        with open(self.file, 'rb') as i:
            assert self.packets == _digest_io(sensor.get_format(self.meta), i)

    def to_json(self) -> str:
        """Serialize to json."""
        return json.dumps(
            {
                'file': self.file,
                'meta': json.loads(str(self.meta)),
                'packets': [asdict(p) for p in self.packets]
            },
            indent=4)

    @classmethod
    def from_bufstream(cls, file: str, meta: sensor.SensorInfo,
                       bufstream: BinaryIO) -> 'StreamDigest':
        """Generate a digest from a bufstream of packets."""
        return cls(file, meta, _digest_io(sensor.get_format(meta), bufstream))

    @classmethod
    def from_json(cls, s: str) -> 'StreamDigest':
        """Instantiate from json representation."""
        d = json.loads(s)
        return cls(file=d['file'],
                   meta=sensor.parse_metadata(json.dumps(d['meta'])),
                   packets=[ScanDigest(**args) for args in d['packets']])
