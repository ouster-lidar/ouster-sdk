"""Utilities for hashing and comparing lidardata."""
from dataclasses import dataclass
from hashlib import md5
import json
from typing import Any, BinaryIO, Dict, List, Iterable, Iterator

from more_itertools import side_effect
import numpy as np

from . import _bufstream as bufstream
from . import (LidarPacket, LidarScan, ChanField, ColHeader, Packet, Packets,
               PacketSource, SensorInfo, Scans)


def _md5(a: np.ndarray) -> str:
    """Get md5 hash of a numpy array."""
    return md5(a.tobytes()).hexdigest()


class LidarBufStream(PacketSource):
    """Read a lidar data stream from a simple binary format used for testing.

    Note: these packet sources will not have any IMU data.
    """
    _metadata: SensorInfo

    def __init__(self, bin: BinaryIO, meta: SensorInfo):
        self._bin = bin
        self._metadata = meta
        ...

    def __iter__(self) -> Iterator[LidarPacket]:
        self._bin.seek(0)
        for buf in bufstream.read(self._bin):
            yield LidarPacket(buf, self._metadata)

    @property
    def metadata(self):
        return self._metadata

    def close(self):
        self._bin.close()


def write_lidar_bufstream(file: str, packets: Iterable[LidarPacket]) -> None:
    """Write lidar packets to a bufstream."""
    with open(file, 'wb') as fo:
        bufstream.write(fo, map(lambda p: p._data, packets))


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

    def check(self, other: 'ScanDigest'):
        for k, v in self.hashes.items():
            assert other.hashes.get(k) == v, f"Match failure key: {k}"

    @classmethod
    def from_packet(cls, p: LidarPacket) -> 'ScanDigest':
        hashes = {}
        hashes.update({c.name: _md5(p.field(c)) for c in ChanField})
        hashes.update({h.name: _md5(p.header(h)) for h in ColHeader})

        return cls(**hashes)

    @classmethod
    def from_scan(cls, ls: LidarScan) -> 'ScanDigest':
        hashes = {}
        hashes.update({c.name: _md5(ls.field(c)) for c in ChanField})

        hashes['TIMESTAMP'] = _md5(
            ls.header(ColHeader.TIMESTAMP).astype(np.uint64))
        hashes['ENCODER_COUNT'] = _md5(
            ls.header(ColHeader.ENCODER_COUNT).astype(np.uint64))
        hashes['STATUS'] = _md5(
            ls.header(ColHeader.STATUS).astype(np.uint64))

        return cls(**hashes)


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
    meta: SensorInfo
    packets: List[ScanDigest]
    scans: List[ScanDigest]

    def check(self, other: 'StreamDigest'):
        """Check that this digest is compatible with another.

        Assert that for each packet and scan recorded in this digest, all
        hashes match hashes in the other digest. The other digest may have
        additional hashes for fields that aren't present here.
        """

        assert len(self.packets) == len(other.packets)
        assert len(self.scans) == len(other.scans)

        for p, q in zip(self.packets, other.packets):
            p.check(q)

        for s, t in zip(self.scans, other.scans):
            s.check(t)

    def to_json(self) -> str:
        """Serialize to json."""
        return json.dumps(
            {
                'meta': json.loads(str(self.meta)),
                'packets': [d.hashes for d in self.packets],
                'scans': [d.hashes for d in self.scans]
            },
            indent=4)

    @classmethod
    def from_packets(cls, source: PacketSource) -> 'StreamDigest':
        """Generate a digest from a packet stream."""
        packet_digests = []

        def append_to_digest(p: Packet):
            if isinstance(p, LidarPacket):
                packet_digests.append(ScanDigest.from_packet(p))

        packets = Packets(side_effect(append_to_digest, source),
                          source.metadata)
        scan_digests = list(map(ScanDigest.from_scan, Scans(packets)))

        return cls(source.metadata, packet_digests, scan_digests)

    @classmethod
    def from_json(cls, json_data: str) -> 'StreamDigest':
        """Instantiate from json representation."""
        d = json.loads(json_data)
        return cls(
            meta=SensorInfo(json.dumps(d['meta'])),
            packets=[ScanDigest(**hashes) for hashes in d.get('packets', [])],
            scans=[ScanDigest(**hashes) for hashes in d.get('scans', [])])
