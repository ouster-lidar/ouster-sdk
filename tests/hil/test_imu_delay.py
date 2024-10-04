from contextlib import closing
import logging
import pytest
import time

from ouster.sdk import client
from ouster.sdk.client import LidarMode, TimestampMode, UDPProfileLidar, PacketFormat

logger = logging.getLogger("HIL")

# PTP uses the TAI offset in all well formed profiles.
# time.time_ns() returns a UNIX timestamp which has leapseconds,
# so compensate here.
TAI_OFFSET_s = 37
TAI_OFFSET_ns = TAI_OFFSET_s * 1e9


@pytest.fixture
def lidar_port():
    return 7502


@pytest.fixture
def imu_port():
    return 7503


@pytest.fixture
def timestamp_mode():
    return TimestampMode.TIME_FROM_PTP_1588


@pytest.fixture
def lidar_mode():
    return LidarMode.MODE_1024x10


@pytest.fixture
def azimuth_window():
    return (0, 360000)


@pytest.fixture
def udp_profile_lidar():
    return UDPProfileLidar.PROFILE_LIDAR_LEGACY


@pytest.fixture
def hil_sensor_config(hil_initial_config, lidar_port, imu_port, lidar_mode,
                      azimuth_window, udp_profile_lidar,
                      timestamp_mode) -> None:
    logger.debug(f"lidar port: {lidar_port} imu_port: {imu_port}")
    hil_initial_config.udp_port_lidar = lidar_port
    hil_initial_config.udp_port_imu = imu_port
    hil_initial_config.lidar_mode = lidar_mode
    hil_initial_config.azimuth_window = azimuth_window
    hil_initial_config.udp_profile_lidar = udp_profile_lidar
    hil_initial_config.timestamp_mode = timestamp_mode
    return hil_initial_config


def test_imu_packets_delay(hil_configured_sensor, lidar_port,
                           imu_port) -> None:
    """Check that the average imu packets delay does not exceed a certain threshold."""

    if hil_configured_sensor is None:
        pytest.fail("Test fails due to rejected configuration")

    warm_up = 10
    sample_count = 100
    total_capture_count = warm_up + sample_count
    minimum_avg_delay_ms = 1.0
    maximum_avg_delay_ms = 5.0
    delays_ms = [0.0] * total_capture_count
    logger.debug(F"Capturing {total_capture_count} imu packets...")
    with closing(client.Sensor(hil_configured_sensor, lidar_port, imu_port)) as \
            sensor:
        pf = PacketFormat(sensor.metadata)
        i = 0
        it = iter(sensor)
        while i < total_capture_count:
            p = next(it)
            if isinstance(p, client.ImuPacket):
                delays_ms[i] = (time.time_ns() -
                                (pf.imu_gyro_ts(p.buf) - TAI_OFFSET_ns)) * 1e-6
                i += 1
    avg = sum(delays_ms[warm_up:]) / float(sample_count)
    assert minimum_avg_delay_ms < avg < maximum_avg_delay_ms
