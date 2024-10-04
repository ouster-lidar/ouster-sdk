from contextlib import closing
import logging
import pytest
import time

from ouster.sdk import client
from ouster.sdk.client import LidarMode, TimestampMode, UDPProfileLidar, PacketFormat, ColHeader

logger = logging.getLogger("HIL")

# PTP uses the TAI offset in all well formed profiles.
# time.time_ns() returns a UNIX timestamp which has leapseconds, compensate here.
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


@pytest.fixture(scope='module',
                params=[
                    pytest.param(LidarMode.MODE_512x10, id="MODE_512x10"),
                    pytest.param(LidarMode.MODE_512x20, id="MODE_512x20"),
                    pytest.param(LidarMode.MODE_1024x10, id="MODE_1024x10"),
                    pytest.param(LidarMode.MODE_1024x20, id="MODE_1024x20"),
                    pytest.param(LidarMode.MODE_2048x10, id="MODE_2048x10"),
                    pytest.param(LidarMode.MODE_4096x5, id="MODE_4096x5")
                ])
def lidar_mode(request):
    return request.param


@pytest.fixture
def azimuth_window():
    return (0, 360000)


@pytest.fixture(
    scope='module',
    params=[
        pytest.param(UDPProfileLidar.PROFILE_LIDAR_LEGACY,
                     id="PROFILE_LIDAR_LEGACY"),
        pytest.param(UDPProfileLidar.PROFILE_LIDAR_RNG15_RFL8_NIR8,
                     id="PROFILE_LIDAR_RNG15_RFL8_NIR8"),
        pytest.param(UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16,
                     id="PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16"),
        pytest.param(UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL,
                     id="PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL"),
        pytest.param(UDPProfileLidar.PROFILE_LIDAR_FUSA_RNG15_RFL8_NIR8_DUAL,
                     id="PROFILE_LIDAR_FUSA_RNG15_RFL8_NIR8_DUAL")
    ])
def udp_profile_lidar(request):
    return request.param


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


def test_lidar_packets_delay(hil_configured_sensor, hil_sensor_config,
                             hil_sensor_firmware, lidar_port,
                             lowest_4096_fw) -> None:
    """Check that the average lidar packets delay does not exceed a certain threshold."""

    if hil_sensor_firmware < lowest_4096_fw \
            and hil_sensor_config.lidar_mode == LidarMode.MODE_4096x5:
        # 4096x5 mode added in MIN_4096_FW
        logger.debug(
            "Skipping 4096x5 sensor delay test on FW {hil_sensor_firmware}")
        return

    if hil_configured_sensor is None:
        pytest.fail("Test fails due to rejected configuration")

    time.sleep(2) # sleep 2 seconds

    warm_up = 1000
    sample_count = 1000
    total_capture_count = warm_up + sample_count
    minimum_avg_delay_ms = 0.0
    maximum_avg_delay_ms = 20.0
    delays_ms = [float] * total_capture_count
    logger.debug(f"Capturing {total_capture_count} lidar packets...")

    # Specify 7505 != imu_port() to make sure we get only lidar packets
    with closing(client.Sensor(hil_configured_sensor, lidar_port,
                               7505, _flush_frames = 10)) as sensor:
        pf = PacketFormat(sensor.metadata)
        for i, p in zip(range(total_capture_count), sensor):
            delays_ms[i] = (time.time_ns() -
                            (pf.packet_header(ColHeader.TIMESTAMP, p.buf)[0] - TAI_OFFSET_ns)) * 1e-6
    avg = sum(delays_ms[warm_up:]) / float(sample_count)  # type: ignore
    assert minimum_avg_delay_ms < avg < maximum_avg_delay_ms
