#  type: ignore
from typing import (Iterator, List, Optional, Tuple, Callable)

import os
import logging
from math import ceil

from threading import Thread

import ouster.sdk.client as client
from ouster.sdk.client import PacketMultiSource
import ouster.sdk.client._client as _client
from ouster.sdk.client import SensorInfo, PacketIdError
from ouster.sdk.client.data import Packet, LidarPacket, ImuPacket


logger = logging.getLogger("multi-logger")

MULTI_DEBUG = 0
try:
    MULTI_DEBUG = int(os.getenv("OUSTER_SDK_MULTI_DEBUG", 0))
    if MULTI_DEBUG:
        logger.setLevel(logging.DEBUG)
except Exception:
    pass


class SensorMultiPacketReader(PacketMultiSource):
    """Multi sensor packet source"""

    def __init__(self,
                 hostnames: List[str],
                 ports: List[Tuple[int, int]],
                 *,
                 buf_size_secs: float = 2.0,
                 timeout: Optional[float] = 2.0,
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
