#  type: ignore
from typing_extensions import Protocol
from typing import (Iterator, List, Optional, Tuple, Union, Callable)

import os
import time
import logging
import copy
from math import ceil

from threading import Thread
from threading import Lock

import ouster.client as client
import ouster.client._client as _client
from ouster.client import (SensorInfo, LidarScan, PacketIdError, packet_ts)
from ouster.client import ScanSource
from ouster.client.data import Packet, LidarPacket, ImuPacket
import ouster.pcap._pcap as _pcap

from functools import partial
from ouster.pcap.pcap import _guess_ports, _packet_info_stream
from ouster.sdkx.util import resolve_extrinsics

from ouster.osf._osf import MessageRef
from ouster.pcap._pcap import PcapIndex


logger = logging.getLogger("multi-logger")

MULTI_DEBUG = 0
try:
    MULTI_DEBUG = int(os.getenv("OUSTER_SDK_MULTI_DEBUG", 0))
    if MULTI_DEBUG:
        logger.setLevel(logging.DEBUG)
except Exception:
    pass


class PacketMultiSource(Protocol):
    """Represents a multi-sensor data stream."""

    def __iter__(self) -> Iterator[Tuple[int, Packet]]:
        """A PacketSource supports ``Iterable[Tuple[int, Packet]]``.

        Currently defined explicitly due to:
        https://github.com/python/typing/issues/561
        """
        ...

    @property
    def metadata(self) -> List[SensorInfo]:
        """Metadata associated with the packet streams."""
        ...

    @property
    def is_live(self) -> bool:
        ...

    @property
    def is_seekable(self) -> bool:
        ...

    @property
    def is_indexed(self) -> bool:
        ...

    def restart(self) -> None:
        """Restart playback, only relevant to non-live sources"""
        ...

    def close(self) -> None:
        """Release the underlying resources, if any."""
        ...


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
                 _soft_id_check: bool = False,
                 _resolve_extrinsics: bool = False):
        """Read a single sensor data stream from a single packet capture file.

        Args:
            metadata_paths: List of sensors metadata files
            pcap_path: File path of recorded pcap
            rate: Output packets in real time, if non-zero
            index: Should index the source, may take extra time on startup
            _soft_id_check: if True, don't skip lidar packets buffers on
                            init_id mismatch
            _resolve_extrinsics: if True attempts to find the extrinsics source
                                 and fill source.metadata[i].extrinsic field
        """
        self._metadata = []
        self._metadata_json = []
        self._indexed = index
        self._soft_id_check = _soft_id_check
        self._id_error_count = 0

        self._port_info = dict()

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
                             _raise_on_id_check=not _soft_id_check)),
                    (meta_info.udp_port_imu, ImuPacket)
                ]
                for packet_port, packet_ctor in port_to_packet:
                    if packet_port in self._port_info:
                        raise RuntimeError(
                            f"Port collision: {packet_port}"
                            f" was already used for another stream")
                    self._port_info[packet_port] = dict(ctor=packet_ctor,
                                                        idx=idx)

        self._extrinsics_source = [None] * len(self._metadata)
        if _resolve_extrinsics:
            # Handle extrinsics search, parse and set to the metadata
            ext_results = resolve_extrinsics(data_path=pcap_path,
                                             infos=self._metadata)
            if ext_results and all(ext_results):
                for idx, ext_record in enumerate(ext_results):
                    ext_mat = ext_record[0]
                    ext_source = ext_record[1]
                    self._metadata[idx].extrinsic = ext_mat
                    ext_src = (ext_source if ext_source.startswith("tar:") else
                               os.path.basename(ext_source))
                    self._extrinsics_source[idx] = f"lookup:{ext_src}"

        self._rate = rate
        self._reader = _pcap.IndexedPcapReader(pcap_path, self._metadata)
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
                buf = packet_data.tobytes()

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
                idx = port_info["idx"]
                packet = port_info["ctor"](buf[0:n], self._metadata[idx],
                                           timestamp)
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
    def _index(self) -> PcapIndex:
        with self._lock:
            return self._reader.get_index() if self._reader else None

    def seek(self, offset: int) -> None:
        self._reader.seek(offset)

    # diagnostics
    @property
    def id_error_count(self) -> int:
        return self._id_error_count

    # TODO: Do we need this info
    @property
    def extrinsics_source(self) -> int:
        return self._extrinsics_source

    def restart(self) -> None:
        """Restart playback, only relevant to non-live sources"""
        with self._lock:
            if self._reader:
                self._reader.reset()

    def close(self) -> None:
        """Release Pcap resources. Thread-safe."""
        with self._lock:
            self._reader = None


# TODO: schedule for removal
class PacketMultiWrapper(PacketMultiSource):
    """Wrap PacketSource to the PacketMultiSource interface"""

    def __init__(self,
                 source: Union[client.PacketSource, PacketMultiSource]) -> None:
        self._source = source

    def __iter__(self) -> Iterator[Tuple[int, Packet]]:
        for p in self._source:
            yield (0, p) if isinstance(p, (client.LidarPacket,
                                           client.ImuPacket)) else p

    @property
    def metadata(self) -> List[SensorInfo]:
        """Metadata associated with the packet streams."""
        meta = self._source.metadata
        return [meta] if isinstance(meta, client.SensorInfo) else meta

    def close(self) -> None:
        """Release the underlying resource, if any."""
        self._source.close()

    @property
    def buf_use(self) -> int:
        if hasattr(self._source, "buf_use"):
            return self._source.buf_use
        else:
            return -1


class SensorMultiPacketReader(PacketMultiSource):
    """Multi sensor packet source"""

    def __init__(self,
                 hostnames: List[str],
                 ports: List[Tuple[int, int]],
                 *,
                 buf_size_secs: float = 2.0,
                 timeout: Optional[float] = 2.0,
                 extrinsics_path: Optional[str] = None,
                 _overflow_err: bool = False,
                 _flush_before_read: bool = True,
                 _flush_frames: int = 5,
                 _skip_metadata_beam_validation: bool = False) -> None:
        """
        Neither the ports nor udp destination configuration on the sensors will
        be updated. The metadata will be fetched over the network from the
        sensors.

        Args:
            hostnames: list of hostnames or IP addresss of the sensors
            ports: list of tuples of UDP ports to listen on for lidar/imu data
            buf_size_secs: seconds of the data to buffer before OVERFLOW
            timeout: seconds to wait for packets before signaling error or None
            _overflow_err: if True, raise ClientOverflow
            _flush_before_read: if True, try to clear buffers before reading
            _skip_metadata_beam_validation: if True, skip metadata beam angle check

        Raises:
            ClientError: If initializing the client fails.
        """
        assert len(hostnames) == len(ports)
        self._hostnames = hostnames
        self._connections = [
            _client.SensorConnection(h, lp, ip)
            for h, (lp, ip) in zip(self._hostnames, ports)
        ]
        self._timeout = timeout
        self._overflow_err = _overflow_err
        self._flush_before_read = _flush_before_read
        self._fetched_meta: List[str] = []
        self._flush_frames = _flush_frames
        self._skip_metadata_beam_validation = _skip_metadata_beam_validation

        # Fetch metadatas from the sensors (always)
        self._fetch_metadata()
        self._metadata = [
            SensorInfo(m_json, self._skip_metadata_beam_validation)
            for m_json in self._fetched_meta
        ]

        self._id_error_count = [0] * len(self.metadata)

        # set names
        for m, hn in zip(self._metadata, self._hostnames):
            m.hostname = hn

        # resolving extrinsics
        self._extrinsics_source = [None] * len(self._metadata)
        logger.debug(f"{extrinsics_path =}")
        if extrinsics_path:
            # Handle extrinsics search, parse and set to the metadata
            ext_results = resolve_extrinsics(data_path=extrinsics_path,
                                             infos=self._metadata)
            if ext_results:
                for idx, ext_record in enumerate(ext_results):
                    if ext_record:
                        ext_mat, ext_source = ext_record
                        self._metadata[idx].extrinsic = ext_mat
                        ext_src = (ext_source if ext_source.startswith("tar:")
                                   else os.path.basename(ext_source))
                        self._extrinsics_source[idx] = f"lookup:{ext_src}"
                        self._extrinsics_source[idx] = ext_src

        self._pf = [_client.PacketFormat.from_info(m) for m in self._metadata]
        self._cli = _client.UDPPacketSource()
        for conn, info in zip(self._connections, self._metadata):
            self._cli.add_client(conn, info, buf_size_secs)

        self._producer = Thread(target=self._cli.produce)
        self._producer.start()

    def _fetch_metadata(self, timeout: Optional[float] = None) -> None:
        timeout_sec = 45
        if timeout:
            timeout_sec = ceil(timeout)
        if not self._fetched_meta:
            self._fetched_meta = [c.get_metadata(
                legacy=False, timeout_sec=timeout_sec) for c in self._connections]
            if not all(self._fetched_meta):
                raise client.ClientError("Failed to collect metadata. UPS :(")

    def _next_packet(self) -> Optional[Tuple[int, Packet]]:
        e = self._cli.pop(self._timeout)
        try:
            if e.state & _client.ClientState.OVERFLOW:
                if self._overflow_err:
                    raise client.ClientOverflow(
                        f"Overflow on source id: {e.source}")
                # TODO[pb]: This is a strange case, not sure what we need to do here ...
                raise ValueError(
                    f"Overflow on sensor [{e.source}] was detected "
                    f"but ClientOverflow can't be raised so we are "
                    f"raising ValueError, hmmm ...")
            if e.state & _client.ClientState.LIDAR_DATA:
                p = self._cli.packet(e)
                packet = LidarPacket(
                    p._data, self._metadata[e.source], p.capture_timestamp)
                return (e.source, packet)
            elif e.state & _client.ClientState.IMU_DATA:
                p = self._cli.packet(e)
                packet = ImuPacket(
                    p._data, self._metadata[e.source], p.capture_timestamp)
                return (e.source, packet)
            elif e.state == _client.ClientState.TIMEOUT:
                raise client.ClientTimeout(
                    f"No packets received within {self._timeout}s")
            elif e.state & _client.ClientState.ERROR:
                raise client.ClientError("Client returned ERROR state")
            elif e.state & _client.ClientState.EXIT:
                return None
        except PacketIdError as err:
            self._id_error_count[e.source] += 1
            raise err
        finally:
            # LidarPacket/ImuPacket ctors may raise but we always want to
            # advance the subscriber so to not overflow
            self._cli.advance(e)

        raise AssertionError("Should be unreachable, UUPS")

    def __iter__(self) -> Iterator[Tuple[int, Packet]]:
        """Starts the multi packet source iterator."""

        if not self._producer.is_alive():
            raise ValueError("I/O operation on closed packet source")

        if self._flush_before_read:
            flush = self._flush_impl(self._flush_frames)
        else:
            # autopep8: off
            flush = lambda _: False
            # autopep8: on

        while True:
            try:
                p = self._next_packet()
                if p is not None:
                    if not flush(p):
                        yield p
                else:
                    break
            except (ValueError, PacketIdError):
                # bad packet size here: this can happen when
                # packets are buffered by the OS, not necessarily an error
                # same pass as in data.py
                pass

    def _flush_impl(
            self,
            n_frames: int = 3) -> Callable[[Tuple[int, client.Packet]], bool]:
        """Makes a flush function for an iterator to drop n_frames per sensor.

        Args:
            n_frames: number of frames to drop (used for every sensor)

        Returns:
            The predicate function that indicate the need to flush the current packet.
        """

        frames_cnt = [n_frames] * len(self.metadata)
        sensor_flushed = [False] * len(self.metadata)

        frame_bound = [client.FrameBorder() for _ in self.metadata]

        def flush_impl(p: Tuple[int, client.Packet]) -> bool:
            nonlocal frame_bound
            idx, packet = p[0], p[1]
            if sensor_flushed[idx]:
                return False
            if not frame_bound[idx](packet):
                return True
            if frames_cnt[idx] > 0:
                frames_cnt[idx] -= 1
                return True
            sensor_flushed[idx] = True
            return False

        return flush_impl

    @property
    def metadata(self) -> List[SensorInfo]:
        """Metadata associated with the packet streams."""
        return self._metadata

    @property
    def is_live(self) -> bool:
        return True

    @property
    def is_seekable(self) -> bool:
        return False

    @property
    def is_indexed(self) -> bool:
        return False

    def restart(self) -> None:
        # NOTE[self]: currently we ignore the call for a live sensor but one
        #  could interpret this invocation as a sensor "reinit" command
        pass

    def close(self) -> None:
        """Shut down producer thread and close network connections.

        Attributes may be unset if constructor throws an exception.
        """
        if hasattr(self, '_cli'):
            self._cli.shutdown()
        if hasattr(self, '_producer'):
            self._producer.join()
        if hasattr(self, '_connections'):
            for conn in self._connections:
                conn.shutdown()

    def __del__(self) -> None:
        self.close()

    # these methods are for diagnostics
    @property
    def buf_use(self) -> int:
        """Size of the buffers that is actially used"""
        return self._cli.size

    @property
    def id_error_count(self) -> List[int]:
        """Number of PacketIdError accumulated per connection/sensor"""
        return self._id_error_count


def collate_scans(
    source: Iterator[Tuple[int, Union[LidarScan, MessageRef]]],
    sensors_count: int,
    get_ts: Callable[[Union[LidarScan, MessageRef]], int],
    *,
    dt: int = 10**8
) -> Iterator[List[Optional[LidarScan]]]:
    """Collate by sensor idx with a cut every `dt` (ns) time length.

    Assuming that multi sensor packets stream are PTP synced, so the sensor
    time of LidarScans don't have huge deltas in time, though some latency
    of packets receiving (up to dt) should be ok.

    Args:
        source: data stream with scans
        sensors_count: number of sensors generating the stream of scans
        dt: max time difference between scans in the collated scan (i.e.
            time period at which every new collated scan is released/cut),
            default is 0.1 s
    Returns:
        List of LidarScans elements
    """
    min_ts = -1
    max_ts = -1
    collated = [None] * sensors_count
    for idx, m in source:
        ts = get_ts(m)
        if min_ts < 0 or max_ts < 0 or (
                ts >= min_ts + dt or ts < max_ts - dt):
            if any(collated):
                # process collated (reached dt boundary, if used)
                yield collated
                collated = [None] * sensors_count

            min_ts = max_ts = ts

        if collated[idx]:
            # process collated (reached the existing scan)
            yield collated
            collated = [None] * sensors_count
            min_ts = max_ts = ts

        collated[idx] = m

        if ts < min_ts:
            min_ts = ts

        if ts > max_ts:
            max_ts = ts

    # process the last one
    if any(collated):
        # process collated (the very last one, if any)
        yield collated


class ScansMulti(client.MultiScanSource):
    """Multi LidarScan source."""

    def __init__(
        self,
        source: PacketMultiSource,
        *,
        dt: int = 10**8,
        complete: bool = False,
        cycle: bool = False,
        fields: Optional[List[client.FieldTypes]] = None,
        **_
    ) -> None:
        """
        Args:
            source: packet multi source
            dt: max time difference between scans in the collated scan (i.e.
                time period at which every new collated scan is released/cut),
                default is 0.1s
            complete: set to True to only release complete scans
            cycle: repeat infinitely after iteration is finished is True.
                    in case source refers to a live sensor then this parameter
                    has no effect.
            fields: specify which channel fields to populate on LidarScans
        """
        self._source = source
        self._dt = dt
        self._complete = complete
        self._cycle = cycle
        # NOTE[self]: this fields override property may need to double checked
        # for example, what would happen if the length of override doesn't
        # match with the actual underlying metadata size. Is this a supported
        # behavior? For now throwing an error if they don't match in size.
        file_fields = [client.get_field_types(
            sinfo) for sinfo in self._source.metadata]
        if fields:
            if len(fields) != len(file_fields):
                raise ValueError("Size of Field override doens't match")
            self._fields = fields
        else:
            self._fields = file_fields

    @property
    def sensors_count(self) -> int:
        return len(self._source.metadata)

    @property
    def metadata(self) -> List[client.SensorInfo]:
        return self._source.metadata

    @property
    def is_live(self) -> bool:
        return self._source.is_live

    @property
    def is_seekable(self) -> bool:
        return self._source.is_seekable

    @property
    def is_indexed(self) -> bool:
        return self._source.is_indexed

    @property
    def fields(self) -> List[client.FieldTypes]:
        return self._fields

    @property
    def scans_num(self) -> List[int]:
        if self.is_live or not self.is_indexed:
            return [0] * self.sensors_count
        raise NotImplementedError

    def __len__(self) -> int:
        if self.is_live or not self.is_indexed:
            return 0
        raise NotImplementedError

    def __iter__(self) -> Iterator[List[Optional[LidarScan]]]:
        return collate_scans(self._scans_iter(True, self._cycle, True), self.sensors_count,
                             client.first_valid_packet_ts,
                             dt=self._dt)

    def _scans_iter(self, restart=True, cycle=False, deep_copy=False
                    ) -> Iterator[Tuple[int, LidarScan]]:
        """
        Parameters:
            restart: restart source from beginning if applicable
            cycle: when reaching end auto restart
            deep_copy: perform deepcopy when yielding scans
        """
        w = [int] * self.sensors_count
        h = [int] * self.sensors_count
        col_window = [int] * self.sensors_count
        columns_per_packet = [int] * self.sensors_count
        pf = [None] * self.sensors_count
        ls_write = [None] * self.sensors_count
        batch = [None] * self.sensors_count

        for i, sinfo in enumerate(self.metadata):
            w[i] = sinfo.format.columns_per_frame
            h[i] = sinfo.format.pixels_per_column
            col_window[i] = sinfo.format.column_window
            columns_per_packet[i] = sinfo.format.columns_per_packet
            pf[i] = client._client.PacketFormat.from_info(sinfo)
            batch[i] = client._client.ScanBatcher(w[i], pf[i])

        # autopep8: off
        scan_shallow_yield = lambda x: x
        scan_deep_yield = lambda x: copy.deepcopy(x)
        scan_yield_op = scan_deep_yield if deep_copy else scan_shallow_yield
        # autopep8: on

        if restart:
            self._source.restart()  # start from the beginning
        while True:
            for idx, packet in self._source:
                if isinstance(packet, client.LidarPacket):
                    ls_write[idx] = ls_write[idx] or client._client.LidarScan(
                        h[idx], w[idx], self._fields[idx], columns_per_packet[idx])
                    if batch[idx](packet._data, packet_ts(packet), ls_write[idx]):
                        if self._complete and not ls_write[idx].complete(col_window[idx]):
                            ls_write[idx] = None
                        yield idx, scan_yield_op(ls_write[idx])

            # TODO[UN]: revisit this piece
            # return the last not fully cut scans in the sensor timestamp order if
            # they satisfy the completeness criteria
            last_scans = sorted(
                [(idx, ls) for idx, ls in enumerate(ls_write) if ls is not None],
                key=lambda si: client.first_valid_packet_ts(si[1]))
            while last_scans:
                idx, ls = last_scans.pop(0)
                if not self._complete or ls.complete(col_window[idx]):
                    yield idx, scan_yield_op(ls)

            if cycle:
                self._source.restart()
            else:
                break

    def _seek(self, offset: int) -> None:
        if not self.is_seekable:
            raise RuntimeError("can not invoke _seek on non-seekable source")
        self._source.seek(offset)

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[List[Optional[LidarScan]], List[List[Optional[LidarScan]]]]:

        if not self.is_indexed:
            raise RuntimeError(
                "can not invoke __getitem__ on non-indexed source")
        raise NotImplementedError

    def set_playback_speed(self, int) -> None:
        raise NotImplementedError

    def close(self) -> None:
        if self._source:
            self._source.close()
            self._source = None

    def __del__(self) -> None:
        self.close()

    def single_source(self, stream_idx: int) -> ScanSource:
        from ouster.client.scan_source_adapter import ScanSourceAdapter
        return ScanSourceAdapter(self, stream_idx)
