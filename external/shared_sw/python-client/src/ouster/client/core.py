"""Ouster sensor python client.

This module is WIP and will contain more idiomatic wrappers around the
lower-level pyblind11-generated module.
"""
from typing import Callable, Generator, Optional, Union
from threading import Event, Thread
from queue import Queue

from . import _sensor
from ._sensor import Client, ClientState, PacketFormat, SensorInfo

from .lidardata import BufferT, ImuPacket, LidarPacket, LidarScan


class ClientError(Exception):
    pass


def batch_to_scan(
        si: SensorInfo) -> Callable[[BufferT], Optional[_sensor.LidarScan]]:
    """Create a function to batch packet buffers to a LidarScan.

    Args:
        si: Sensor metadata

    Returns:
        Callable: a closure that can be called with a packet buffer.

        Optionally returns a lidar scan, when the added packet is determined to
        be from the next frame.
    """
    w = si.format.columns_per_frame
    h = si.format.pixels_per_column

    ls_write, ls_yield = _sensor.LidarScan(w, h), None

    # when a frame is complete, assign it to ls_yield and start new frame
    def complete_frame(ts: int):
        nonlocal ls_write, ls_yield
        ls_yield = ls_write
        ls_write = _sensor.LidarScan(w, h)

    batch = _sensor.batch_to_scan(w, _sensor.get_format(si), complete_frame)

    # batch another packet buffer and optionally return a completed frame
    def batcher(buf: BufferT) -> Optional[_sensor.LidarScan]:
        nonlocal ls_yield
        ls_yield = None
        batch(buf, ls_write)
        return ls_yield

    return batcher


def packets(
        pf: PacketFormat,
        cli: Client) -> Generator[Union[ImuPacket, LidarPacket], None, None]:
    """Create a stream of packets emitted by a sensor."""
    try:
        while True:
            st = _sensor.poll_client(cli)
            if st & ClientState.ERROR:
                raise ClientError("Client returned error state")
            elif st & ClientState.LIDAR_DATA:
                lidar_buf = bytearray(pf.lidar_packet_size + 1)
                if (_sensor.read_lidar_packet(cli, lidar_buf, pf)):
                    yield LidarPacket(lidar_buf, pf)
            elif st & ClientState.IMU_DATA:
                imu_buf = bytearray(pf.imu_packet_size + 1)
                if (_sensor.read_imu_packet(cli, imu_buf, pf)):
                    yield ImuPacket(imu_buf, pf)

    except KeyboardInterrupt:
        pass


class ScanQueue():
    """Context manager for a producer/consumer queue of lidar scans.

    Launches a thread that batches scans without holding the GIL. The consumer
    can dequeue items and continue to do work without being blocked.
    """
    _queue: Queue

    def __init__(self, cli: Client, size: int = 16):
        self._cli = cli
        self._running = Event()
        self._producer = Thread(target=self._enqueue)
        self._queue = Queue(size)

        si = _sensor.parse_metadata(_sensor.get_metadata(cli))
        pf = _sensor.get_format(si)
        w = si.format.columns_per_frame
        self._next_scan = _sensor.scan_batcher(w, pf)

    def _enqueue(self):
        while self._running.is_set():
            nscan = self._next_scan(self._cli)
            scan = LidarScan.from_buffer(nscan.w, nscan.h, nscan.data)
            self._queue.put(scan, block=True)

    def __enter__(self):
        self._running.set()
        self._producer.start()
        return self._queue

    def __exit__(self, type, value, traceback):
        # stop and wait for the producer thread to exit
        self._running.clear()
        self._producer.join()


def scans(
        cli: Client
) -> Generator[Union[ImuPacket, _sensor.LidarScan], None, None]:
    try:

        si = _sensor.parse_metadata(_sensor.get_metadata(cli))
        pf = _sensor.get_format(si)

        lidar_buf = bytearray(pf.lidar_packet_size + 1)

        batch = batch_to_scan(si)

        while True:
            st = _sensor.poll_client(cli)
            if st & ClientState.ERROR:
                raise ClientError("Client returned error state")
            elif st & ClientState.LIDAR_DATA:
                if (_sensor.read_lidar_packet(cli, lidar_buf, pf)):
                    ls = batch(lidar_buf)
                    if ls is not None:
                        yield ls
            elif st & ClientState.IMU_DATA:
                imu_buf = bytearray(pf.imu_packet_size + 1)
                if (_sensor.read_imu_packet(cli, imu_buf, pf)):
                    yield ImuPacket(imu_buf, pf)

    except KeyboardInterrupt:
        pass
