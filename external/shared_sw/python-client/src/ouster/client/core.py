"""Ouster sensor python client.

This module is WIP and will contain more idiomatic wrappers around the
lower-level pyblind11-generated module.
"""
from typing import Generator

from ._sensor import Client, ClientState, PacketFormat
from ._sensor import poll_client, read_lidar_packet, read_imu_packet


class ClientError(Exception):
    pass


def lidar_packets(pf: PacketFormat,
                  cli: Client) -> Generator[bytearray, None, None]:
    """Create a stream of just the lidar packets emitted by a sensor."""
    try:
        imu_buf = bytearray(pf.imu_packet_size + 1)
        while True:
            st = poll_client(cli)
            if st & ClientState.ERROR:
                raise ClientError("Client returned error state")
            elif st & ClientState.LIDAR_DATA:
                lidar_buf = bytearray(pf.lidar_packet_size + 1)
                if (read_lidar_packet(cli, lidar_buf, pf)):
                    yield lidar_buf
            elif st & ClientState.IMU_DATA:
                read_imu_packet(cli, imu_buf, pf)
    except KeyboardInterrupt:
        pass
