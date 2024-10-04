"""Ouster sensor Python client.

Copyright (c) 2021, Ouster, Inc.
All rights reserved.

This module contains more idiomatic wrappers around the lower-level module
generated using pybind11.
"""
from typing import (Optional, Iterator)
import logging
import warnings

from ouster.sdk._bindings.client import (SensorInfo, Packet, SensorConfig)

from .core import PacketSource

from .multi import SensorPacketSource

logger = logging.getLogger("ouster.sdk.client.sensor")


class Sensor(PacketSource):
    """Deprecated: A packet source listening on local UDP ports for a single sensor."""

    _source: SensorPacketSource

    def __init__(self,
                 hostname: str,
                 lidar_port: Optional[int] = None,
                 imu_port: Optional[int] = None,
                 *,
                 metadata: Optional[SensorInfo] = None,
                 buf_size: float = 0.2,
                 timeout: Optional[float] = 2.0,
                 _overflow_err: bool = False,
                 _flush_before_read: bool = True,
                 _flush_frames: int = 5,
                 soft_id_check: bool = False,
                 _skip_metadata_beam_validation: bool = False) -> None:
        warnings.warn("client.Sensor(...) is deprecated: "
                  "Use client.SensorPacketSource(...).single_source(0) instead. "
                  "This API is planned to be removed in Q4 2024.",
                  DeprecationWarning, stacklevel=2)
        self._source = None  # type: ignore
        config = SensorConfig()
        config.udp_port_lidar = lidar_port
        config.udp_port_imu = imu_port
        self._source = SensorPacketSource([(hostname, config)], metadata=[metadata] if metadata else None,
                                          timeout=timeout, _overflow_err=_overflow_err,
                                          _flush_before_read=_flush_before_read, buf_size=buf_size,
                                          _flush_frames=_flush_frames, soft_id_check=soft_id_check,
                                          _skip_metadata_beam_validation=_skip_metadata_beam_validation)

    @property
    def is_live(self) -> bool:
        return True

    @property
    def lidar_port(self) -> int:
        return self._source.metadata[0].config.udp_port_lidar or 0

    @property
    def imu_port(self) -> int:
        return self._source.metadata[0].config.udp_port_imu or 0

    @property
    def metadata(self) -> SensorInfo:
        return self._source.metadata[0]

    def write_metadata(self, path: str) -> None:
        """Save metadata to disk.

        Args:
            path: path to write
        """
        with open(path, 'w') as f:
            f.write(self._source.metadata[0].to_json_string())

    def __iter__(self) -> Iterator[Packet]:
        """Access the UDP data stream as an iterator.

        Reading may block waiting for network data for up to the specified
        timeout. Failing to consume this iterator faster than the data rate of
        the sensor may cause packets to be dropped. Returned packet is meant to
        be consumed prior to incrementing the iterator, and storing the returned
        packet in a container may result in the contents being invalidated. If
        such behaviour is necessary, deepcopy the packets upon retrieval.

        Raises:
            ClientTimeout: if no packets are received within the configured
                timeout
            ClientError: if the client enters an unspecified error state
            ValueError: if the packet source has already been closed
        """

        it = iter(self._source)
        for p in it:
            yield p[1]

    def flush(self, n_frames: int = 3, *, full=False) -> int:
        """Drop some data to clear internal buffers.

        Args:
            n_frames: number of frames to drop
            full: clear internal buffers first, so data is read from the OS
                  receive buffers (or the network) directly

        Returns:
            The number of packets dropped

        Raises:
            ClientTimeout: if a lidar packet is not received within the
                configured timeout
            ClientError: if the client enters an unspecified error state
        """
        return self._source.flush(n_frames=n_frames, full=full)

    @property
    def buf_use(self) -> int:
        return self._source.buf_use

    @property
    def id_error_count(self) -> int:
        return self._source.id_error_count

    def close(self) -> None:
        """Shut down producer thread and close network connection.

        Attributes may be unset if constructor throws an exception.
        """
        if self._source:
            self._source.close()

    def __del__(self) -> None:
        self.close()
