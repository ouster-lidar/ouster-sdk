from typing import Dict, Iterator, List, Optional, Tuple
from ouster.sdk.client import PacketMultiSource
import ouster.sdk.pcap._pcap as _pcap
from ouster.sdk.pcap._pcap import PcapIndex     # type: ignore
from ouster.sdk.pcap.pcap import _guess_ports, _packet_info_stream
from functools import partial

import time

from threading import Lock

from ouster.sdk.client import SensorInfo, PacketIdError
from ouster.sdk.client.data import Packet, LidarPacket, ImuPacket


class PcapMultiPacketReader(PacketMultiSource):
    """Read a sensors packet streams out of a pcap file as an iterator."""

    _metadata: List[SensorInfo]
    _metadata_json: List[str]
    _rate: float
    _handle: Optional[_pcap.playback_handle]
    _lock: Lock

    def __init__(self,
                 pcap_path: str,
                 metadata_paths: List[str],
                 *,
                 rate: float = 0.0,
                 index: bool = False,
                 soft_id_check: bool = False):
        """Read a single sensor data stream from a single packet capture file.

        Args:
            metadata_paths: List of sensors metadata files
            pcap_path: File path of recorded pcap
            rate: Output packets in real time, if non-zero
            index: Should index the source, may take extra time on startup
            soft_id_check: if True, don't skip lidar packets buffers on
                            init_id mismatch
        """
        self._metadata = []
        self._metadata_json = []
        self._indexed = index
        self._soft_id_check = soft_id_check
        self._id_error_count = 0

        self._port_info: Dict[int, object] = dict()

        # sample pcap and attempt to find UDP ports consistent with metadatas
        # NOTE[pb]: Needed for port guessing logic for old single sensor data.
        n_packets = 1000
        stats = _packet_info_stream(pcap_path, n_packets)

        for meta_path in metadata_paths:
            with open(meta_path) as meta_file:
                meta_json = meta_file.read()
                meta_info = SensorInfo(meta_json)
                self._metadata_json.append(meta_json)
                self._metadata.append(meta_info)
                idx = len(self._metadata) - 1

                # NOTE: Rudimentary logic of port guessing that is still needed
                #       for old single sensor data when `udp_port_lidar` and
                #       `udp_port_imu` fields are not set in sensor metadata.
                #       In some distant future we may need to remove it.
                guesses = _guess_ports(stats, self._metadata[idx])
                if len(guesses) > 0:
                    lidar_guess, imu_guess = guesses[0]
                    meta_info.udp_port_lidar = meta_info.udp_port_lidar or lidar_guess
                    meta_info.udp_port_imu = meta_info.udp_port_imu or imu_guess

                port_to_packet = [
                    (meta_info.udp_port_lidar,
                     partial(LidarPacket,
                             _raise_on_id_check=not soft_id_check)),
                    (meta_info.udp_port_imu, ImuPacket)
                ]
                for packet_port, packet_ctor in port_to_packet:
                    if packet_port in self._port_info:
                        raise RuntimeError(
                            f"Port collision: {packet_port}"
                            f" was already used for another stream")
                    self._port_info[packet_port] = dict(ctor=packet_ctor,
                                                        idx=idx)

        self._rate = rate
        self._reader: Optional[_pcap.IndexedPcapReader] = \
            _pcap.IndexedPcapReader(pcap_path, self._metadata)   # type: ignore
        if self._indexed:
            self._reader.build_index()
        self._lock = Lock()

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

            try:
                port_info = self._port_info[packet_info.dst_port]
                idx = port_info["idx"]      # type: ignore
                packet = port_info["ctor"](buf[0:n], self._metadata[idx], timestamp)    # type: ignore
                if isinstance(packet, LidarPacket) and packet.id_error:
                    self._id_error_count += 1
                yield (idx, packet)
            except PacketIdError:
                self._id_error_count += 1
            except ValueError:
                # bad packet size here: this can happen when
                # packets are buffered by the OS, not necessarily an error
                # same pass as in core.py
                # TODO: introduce status for PacketSource to indicate frequency
                # of bad packet size or init_id/sn errors
                pass

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

    # diagnostics
    @property
    def id_error_count(self) -> int:
        return self._id_error_count

    def restart(self) -> None:
        """Restart playback, only relevant to non-live sources"""
        with self._lock:
            if self._reader:
                self._reader.reset()

    def close(self) -> None:
        """Release Pcap resources. Thread-safe."""
        with self._lock:
            self._reader = None
