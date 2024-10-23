from typing import Dict, Iterator, List, Optional, Tuple, Union
from ouster.sdk.client import PacketMultiSource, UDPProfileLidar, PacketSource
import ouster.sdk._bindings.pcap as _pcap
from ouster.sdk._bindings.pcap import PcapIndex     # type: ignore
from ouster.sdk.pcap.pcap import _guess_ports, _packet_info_stream
from ouster.sdk.util import (resolve_metadata_multi)    # type: ignore

import time
import numpy as np
import os

from threading import Lock
import logging
from ouster.sdk.client import SensorInfo, PacketFormat, PacketValidationFailure, LidarPacket, ImuPacket, Packet

logger = logging.getLogger("pcap-logger")


class PcapDuplicatePortException(Exception):
    def __init__(self, port):
        self._port = port

    def __str__(self):
        result = f"Port collision: {self._port} for legacy"
        result += " was already used for another stream."
        return result


class PcapMultiPacketReader(PacketMultiSource):
    """Read a sensors packet streams out of a pcap file as an iterator."""

    _metadata: List[SensorInfo]
    _metadata_json: List[str]
    _rate: float
    _handle: Optional[_pcap.playback_handle]
    _lock: Lock

    def __init__(self,
                 pcap_path: str,
                 metadata_paths: Optional[List[str]] = None,
                 *,
                 metadatas: Optional[List[SensorInfo]] = None,
                 rate: float = 0.0,
                 index: bool = False,
                 soft_id_check: bool = False):
        """Read a multiple sensor data streams from a single packet capture file.

        Args:
            metadata_paths: List of sensors metadata file paths
            metadatas: List of sensor metadatas
            pcap_path: File path of recorded pcap
            rate: Output packets in real time, if non-zero
            index: Should index the source, may take extra time on startup
            soft_id_check: if True, don't skip lidar packets buffers on
                            init_id mismatch
        """
        if not metadatas:
            if not metadata_paths:
                metadata_paths = resolve_metadata_multi(pcap_path)

            if not metadata_paths:
                raise RuntimeError(
                    "Metadata jsons not found. Make sure that metadata json files "
                    "have common prefix with a PCAP file")

        # TODO: need a better way to save these
        print(f"loading metadata from {metadata_paths}")

        self._metadata = []
        self._metadata_json = []
        self._indexed = index
        self._soft_id_check = soft_id_check
        self._id_error_count = 0
        self._size_error_count = 0
        self._imu_size: Optional[int] = None
        self._port_info: Dict[int, Dict[str, List[Dict]]] = dict()

        # sample pcap and attempt to find UDP ports consistent with metadatas
        # NOTE[pb]: Needed for port guessing logic for old single sensor data.
        n_packets = 1000
        stats = _packet_info_stream(pcap_path, n_packets)

        if len(metadata_paths or []) > 0 and metadatas is not None:
            raise RuntimeError("Cannot provide both metadata and metadata paths")

        # load all metadatas
        if metadatas is None and metadata_paths:
            metadatas = []
            for idx, meta_path in enumerate(metadata_paths):
                meta_path_full = os.path.expanduser(meta_path)
                with open(meta_path_full) as meta_file:
                    meta_json = meta_file.read()
                    meta_info = SensorInfo(meta_json)
                    self._metadata_json.append(meta_json)
                    self._metadata.append(meta_info)
        elif metadatas:
            for m in metadatas:
                self._metadata_json.append(m.to_json_string())
                self._metadata.append(m)

        for idx in range(0, len(self._metadata)):
            meta_info = self._metadata[idx]
            # NOTE: Rudimentary logic of port guessing that is still needed
            #       for old single sensor data when `udp_port_lidar` and
            #       `udp_port_imu` fields are not set in sensor metadata.
            #       In some distant future we may need to remove it.
            guesses = _guess_ports(stats, meta_info)
            if len(guesses) > 0:
                lidar_guess, imu_guess = guesses[0]
                meta_info.config.udp_port_lidar = meta_info.config.udp_port_lidar or lidar_guess
                meta_info.config.udp_port_imu = meta_info.config.udp_port_imu or imu_guess

            lidar_sn = ""
            if meta_info.config.udp_profile_lidar == UDPProfileLidar.PROFILE_LIDAR_LEGACY:
                lidar_sn = "LEGACY_LIDAR"
            else:
                lidar_sn = meta_info.sn
            imu_sn = "LEGACY_IMU"
            port_to_packet = [
                (meta_info.config.udp_port_lidar, lidar_sn),
                (meta_info.config.udp_port_imu, imu_sn)
            ]
            for packet_port, sn in port_to_packet:
                if packet_port in self._port_info:
                    if sn in self._port_info[packet_port] and "LEGACY" in sn:
                        raise PcapDuplicatePortException(packet_port)
                else:
                    if packet_port is None:
                        raise RuntimeError(f"Metadata for {meta_info.sn} was missing port numbers.")
                    self._port_info[packet_port] = dict()
                if sn not in self._port_info[packet_port]:
                    self._port_info[packet_port][sn] = list()
                self._port_info[packet_port][sn].append({'idx': idx})
        self._rate = rate
        self._reader: Optional[_pcap.IndexedPcapReader] = \
            _pcap.IndexedPcapReader(pcap_path, self._metadata)   # type: ignore
        if self._indexed:
            self._reader.build_index()
        self._lock = Lock()
        self._pf = []
        for m in self._metadata:
            pf = PacketFormat(m)
            if self._imu_size is None:
                self._imu_size = pf.imu_packet_size
            else:
                # Update this when imu packet sizes are not the same
                assert self._imu_size == pf.imu_packet_size

            self._pf.append(pf)

    def __iter__(self) -> Iterator[Tuple[int, Packet]]:
        with self._lock:
            if self._reader is None:
                raise ValueError("I/O operation on closed packet source")

        buf = bytearray(2**16)
        packet_info = _pcap.packet_info()

        real_start_ts = time.monotonic()
        pcap_start_ts = None
        while True:
            with self._lock:
                if not (self._reader and
                        self._reader.next_packet()):
                    break
                packet_info = self._reader.current_info()
                if packet_info.dst_port not in self._port_info:
                    # not lidar or imu packet that we are interested in
                    continue
                packet_data = self._reader.current_data()
                n = len(packet_data)
                buf = packet_data.tobytes()     # type: ignore

            # if rate is set, read in 'real time' simulating UDP stream
            # TODO: factor out into separate packet iterator utility
            timestamp = packet_info.timestamp
            if self._rate:
                if not pcap_start_ts:
                    pcap_start_ts = timestamp
                real_delta = time.monotonic() - real_start_ts
                pcap_delta = (timestamp - pcap_start_ts) / self._rate
                delta = max(0, pcap_delta - real_delta)
                time.sleep(delta)

            data_ = np.frombuffer(buf[0:n], dtype=np.uint8, count=n)
            ts_ = int(timestamp * 1e9)
            size_errors = 0
            id_errors = []
            match_found = False
            port_info = self._port_info[packet_info.dst_port]
            packet: Optional[Union[LidarPacket, ImuPacket]] = None
            if n == self._imu_size:
                packet = ImuPacket(n)
            else:
                packet = LidarPacket(n)
            packet.buf[:] = data_
            packet.host_timestamp = ts_
            for sn in port_info:
                for item in port_info[sn]:
                    idx = item['idx']
                    res = packet.validate(self._metadata[idx], self._pf[idx])
                    if res == PacketValidationFailure.NONE:
                        yield (idx, packet)
                        match_found = True
                        break
                    elif res == PacketValidationFailure.PACKET_SIZE:
                        size_errors += 1
                    elif res == PacketValidationFailure.ID:
                        id_errors.append((idx, packet))
                if match_found:
                    break
            if not match_found:
                if len(id_errors) > 0:
                    self._id_error_count += 1
                    if len(id_errors) > 1 and self._soft_id_check:
                        raise RuntimeError("Soft ID Checking Does NOT Work With Multiple Sensors")
                    else:
                        if self._soft_id_check:
                            yield (idx, packet)
                elif size_errors > 0:
                    self._size_error_count += 1

    @property
    def metadata(self) -> List[SensorInfo]:
        """Metadata associated with the packet."""
        return self._metadata

    @property
    def is_live(self) -> bool:
        return False

    @property
    def is_seekable(self) -> bool:
        return self._indexed

    @property
    def is_indexed(self) -> bool:
        return self._indexed

    @property
    def _index(self) -> Optional[PcapIndex]:
        with self._lock:
            return self._reader.get_index() if self._reader else None

    def seek(self, offset: int) -> None:
        if self._reader:
            self._reader.seek(offset)

    def single_source(self, stream_idx: int) -> PacketSource:
        from ouster.sdk.client.packet_source_adapter import PacketSourceAdapter
        return PacketSourceAdapter(self, stream_idx)

    # diagnostics
    @property
    def id_error_count(self) -> int:
        return self._id_error_count

    @property
    def size_error_count(self) -> int:
        return self._size_error_count

    def restart(self) -> None:
        """Restart playback, only relevant to non-live sources"""
        with self._lock:
            if self._reader:
                self._reader.reset()

    def close(self) -> None:
        """Release Pcap resources. Thread-safe."""
        with self._lock:
            self._reader = None

    @property
    def closed(self) -> bool:
        """Check if source is closed. Thread-safe."""
        with self._lock:
            return self._reader is None
