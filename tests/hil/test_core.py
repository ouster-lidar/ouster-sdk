from contextlib import closing
import logging
from os import path
from time import sleep
from conftest import deepcopy_iter
from typing import List, Optional, cast

from more_itertools import take, time_limited
import numpy as np

import json
import requests
import pytest

from ouster.sdk import core, sensor
from ouster.sdk.core import UDPProfileLidar, TimestampMode, LidarMode, PacketFormat, ColHeader
from ouster.sdk._bindings import client as _client

logger = logging.getLogger("HIL")


@pytest.fixture
def lidar_port():
    return 7502


@pytest.fixture
def imu_port():
    return 7503


@pytest.fixture
def lidar_mode():
    return LidarMode.MODE_1024x10

@pytest.fixture
def timestamp_mode():
    return TimestampMode.TIME_FROM_INTERNAL_OSC


@pytest.fixture(
    scope='module',
    params=[
        pytest.param(UDPProfileLidar.PROFILE_LIDAR_LEGACY,
                     id="PROFILE_LIDAR_LEGACY"),
        pytest.param(UDPProfileLidar.PROFILE_LIDAR_RNG15_RFL8_NIR8,
                     id="PROFILE_LIDAR_RNG15_RFL8_NIR8",
                     marks=pytest.mark.full),
        pytest.param(UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16,
                     id="PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16"),
        pytest.param(UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL,
                     id="PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL",
                     marks=pytest.mark.full),
        pytest.param(UDPProfileLidar.PROFILE_LIDAR_FUSA_RNG15_RFL8_NIR8_DUAL,
                     id="PROFILE_LIDAR_FUSA_RNG15_RFL8_NIR8_DUAL",
                     marks=pytest.mark.full)
    ])
def udp_profile_lidar(request):
    return request.param


@pytest.fixture(scope='module',
                params=[
                    pytest.param((0, 360000), id="full_az_window"),
                    pytest.param((20000, 65000),
                                 id="small_az_window",
                                 marks=pytest.mark.full)
                ])
def azimuth_window(request):
    return request.param


@pytest.fixture
def hil_sensor_config(hil_initial_config, lidar_port, imu_port, lidar_mode,
                      azimuth_window, udp_profile_lidar) -> None:
    logger.debug(f"lidar port: {lidar_port} imu_port: {imu_port}")
    hil_initial_config.udp_port_lidar = lidar_port
    hil_initial_config.udp_port_imu = imu_port
    hil_initial_config.lidar_mode = lidar_mode
    hil_initial_config.azimuth_window = azimuth_window
    hil_initial_config.udp_profile_lidar = udp_profile_lidar
    return hil_initial_config


def test_packets_dynamic_port(hil_configured_sensor) -> None:
    """Test that selecting a dynamic port works.

    TODO: currently no easy way to use this to capture packets in Python
    """
    logger.debug("Capturing packets...")

    if hil_configured_sensor is None:
        pytest.fail("Test fails due to rejected configuration")

    with closing(sensor.SensorPacketSource(hil_configured_sensor, lidar_port=0, imu_port=0)), closing(
            sensor.SensorPacketSource(hil_configured_sensor, lidar_port=0, imu_port=0)):
        pass


def test_only_lidar_packets(hil_configured_sensor, lidar_port) -> None:
    """Check that we can read packets when only lidar packets are arriving."""

    if hil_configured_sensor is None:
        pytest.fail("Test fails due to rejected configuration")

    with closing(sensor.SensorPacketSource(hil_configured_sensor)) as src:
        metadata = src.sensor_info

    metadata[0].config.udp_port_imu = 7505
    logger.debug("Capturing packets...")
    with closing(sensor.SensorPacketSource(hil_configured_sensor, timeout=2.0, sensor_info=metadata)) as src:
        packets = take(640, src)

    assert len(packets) == 640
    assert all(isinstance(p, core.LidarPacket) for idx, p in packets)


def test_packets_timeout(hil_configured_sensor) -> None:
    """Check that reading times out when no packets arrive."""

    if hil_configured_sensor is None:
        pytest.fail("Test fails due to rejected configuration")

    with closing(sensor.SensorPacketSource(hil_configured_sensor)) as src:
        metadata = src.sensor_info

    metadata[0].config.udp_port_lidar = 7505
    metadata[0].config.udp_port_imu = 7505
    logger.debug("Capturing packets...")
    with pytest.raises(sensor.ClientTimeout):
        with closing(sensor.SensorPacketSource(hil_configured_sensor, sensor_info=metadata)) as src:
            next(iter(src))


def concat_measurement_ids(packets, pf) -> np.ndarray:
    return np.concatenate([
        pf.packet_header(ColHeader.MEASUREMENT_ID, p.buf) for idx, p in packets if isinstance(p, core.LidarPacket)
    ])


@pytest.mark.parametrize(
    'lidar_mode, azimuth_window',
    [pytest.param(LidarMode.MODE_2048x10, (0, 360000), id="full_2048")])
def test_packets_consecutive(hil_configured_sensor) -> None:
    """Check that packets are not being dropped."""
    if hil_configured_sensor is None:
        pytest.fail("Test fails due to rejected configuration")

    logger.debug("Capturing packets for 10s...")
    with closing(sensor.SensorPacketSource(hil_configured_sensor, timeout=2.0, buffer_time_sec=2.0)) as src:
        print("Final config:", src.sensor_info[0].config, "\n\n")
        w = src.sensor_info[0].format.columns_per_frame
        take(640, src) # flush some
        packets = list(time_limited(10, src))
    logger.debug("Done capturing")

    mids = concat_measurement_ids(packets, PacketFormat(src.sensor_info[0])).astype(np.int64)
    assert np.count_nonzero(
        np.diff(mids) % w != 1) == 0, "Got non-consecutive measurements"


@pytest.mark.parametrize('lidar_port, imu_port', [(7504, 7505)])
def test_packets_nonstandard_port(hil_configured_sensor, lidar_port,
                                  imu_port) -> None:
    """Test that we can read packets on a nonstandard port."""

    if hil_configured_sensor is None:
        pytest.fail("Test fails due to rejected configuration")

    logger.debug("Capturing packets...")
    with closing(sensor.SensorPacketSource(hil_configured_sensor, lidar_port=lidar_port,
                               imu_port=imu_port)) as src:
        packets = take(640, src)

    assert len(packets) == 640


def fix_float_rounding(metadata, src):
    def check_close(left, right):
        return np.isclose(left, right).all()
    # fix floating point errors
    if check_close(metadata.beam_azimuth_angles, src.sensor_info[0].beam_azimuth_angles):
        metadata.beam_azimuth_angles = src.sensor_info[0].beam_azimuth_angles
    if check_close(metadata.beam_altitude_angles, src.sensor_info[0].beam_altitude_angles):
        metadata.beam_altitude_angles = src.sensor_info[0].beam_altitude_angles
    if check_close([metadata.lidar_origin_to_beam_origin_mm], [src.sensor_info[0].lidar_origin_to_beam_origin_mm]):
        metadata.lidar_origin_to_beam_origin_mm = src.sensor_info[0].lidar_origin_to_beam_origin_mm
    if check_close(metadata.beam_to_lidar_transform, src.sensor_info[0].beam_to_lidar_transform):
        metadata.beam_to_lidar_transform = src.sensor_info[0].beam_to_lidar_transform
    if check_close(metadata.imu_to_sensor_transform, src.sensor_info[0].imu_to_sensor_transform):
        metadata.imu_to_sensor_transform = src.sensor_info[0].imu_to_sensor_transform
    if check_close(metadata.lidar_to_sensor_transform, src.sensor_info[0].lidar_to_sensor_transform):
        metadata.lidar_to_sensor_transform = src.sensor_info[0].lidar_to_sensor_transform
    if check_close(metadata.extrinsic, src.sensor_info[0].extrinsic):
        metadata.extrinsic = src.sensor_info[0].extrinsic


def test_sensor_metadata_endpoint(hil_configured_sensor, tmpdir, hil_sensor_firmware, lowest_metadata_endpoint_fw) -> None:
    """Test that SensorInfo matches when from sensor http endpoint"""

    if hil_configured_sensor is None:
        pytest.fail("Test fails due to rejected configuration")

    hostname = hil_configured_sensor

    if hil_sensor_firmware < lowest_metadata_endpoint_fw:
        pytest.skip(f"Skip sensor metadata endpoint test as FW version {hil_sensor_firmware} lt {lowest_metadata_endpoint_fw} which added the metadata endpoint")

    with closing(sensor.SensorPacketSource(hil_configured_sensor)) as src:
        http_metadata_endpoint = "http://" + hostname + "/api/v1/sensor/metadata"
        http_userdata_endpoint = "http://" + hostname + "/api/v1/user/data"

        metadata_response = requests.get(http_metadata_endpoint)
        metadata = core.SensorInfo(core.SensorInfo(metadata_response.text).to_json_string())

        userdata_response = requests.get(http_userdata_endpoint)
        if userdata_response.status_code == 200:
            metadata.user_data = json.loads(userdata_response.text)

        fix_float_rounding(metadata, src)
        # TWS 20230818: the decoded fields should be equal
        assert metadata.user_data == src.sensor_info[0].user_data
        assert metadata.has_fields_equal(src.sensor_info[0])
        assert type(metadata) == type(src.sensor_info[0])

        meta_via_requests = json.loads(metadata_response.text)
        meta_via_curl = json.loads(src.sensor_info[0].to_json_string())

        assert 'ouster-sdk' not in meta_via_requests
        # ouster-sdk.client_version is added in the `collect_metadata` method!
        assert 'ouster-sdk' in meta_via_curl

        assert metadata == src.sensor_info[0]


def n_frame_id_gaps(ss: List[List[Optional[core.LidarScan]]]) -> int:
    """Return number of non-consecutive frame ids."""
    return np.count_nonzero(np.diff([cast(core.LidarScan, s).frame_id for s, in ss]) % 2**16 != 1)


def test_scans_consecutive(hil_configured_sensor) -> None:
    """Test that we can batch complete consecutive scans."""

    if hil_configured_sensor is None:
        pytest.fail("Test fails due to rejected configuration")

    with closing(sensor.SensorScanSource(hil_configured_sensor, timeout=2.0)) as scans:
        take(10, scans) # flush
        ss = take(10, scans)

    assert all(s is not None and s.complete() for s, in ss), "Received incomplete scans"
    assert n_frame_id_gaps(ss) == 0, "Gap in frame ids"


def test_scans_read_gap(hil_configured_sensor) -> None:
    """Test that sleeping while reading scans causes a single gap"""

    if hil_configured_sensor is None:
        pytest.fail("Test fails due to rejected configuration")

    with closing(sensor.SensorScanSource(hil_configured_sensor, timeout=2.0)) as scans:
        take(10, scans) # flush
        ss = take(10, scans)
        logger.debug("Pausing during reading scans...")
        sleep(1.0)
        ss += take(10, scans)

    assert len(ss) == 20
    assert all(s is not None and s.complete() for s, in ss), "Received incomplete scans"
    assert n_frame_id_gaps(ss) == 1, "Did not get exactly one gap in frame ids"


def test_scans_read_timeout(hil_configured_sensor) -> None:
    """Test timeout reading scans."""

    if hil_configured_sensor is None:
        pytest.fail("Test fails due to rejected configuration")

    with closing(sensor.SensorPacketSource(hil_configured_sensor)) as src:
        metadata = src.sensor_info

    metadata[0].config.udp_port_imu = 7504
    metadata[0].config.udp_port_lidar = 7504
    with closing(sensor.SensorScanSource(hil_configured_sensor, sensor_info=metadata, timeout=1.0)) as scans:
        with pytest.raises(sensor.ClientTimeout):
            next(iter(scans))


def test_scans_read_timeout_only_imu(hil_configured_sensor, imu_port) -> None:
    """Test timeout reading scans when only IMU packets are arriving."""

    if hil_configured_sensor is None:
        pytest.fail("Test fails due to rejected configuration")

    with closing(sensor.SensorPacketSource(hil_configured_sensor)) as src:
        metadata = src.sensor_info

    metadata[0].config.udp_port_lidar = 7504
    with closing(sensor.SensorScanSource(hil_configured_sensor, sensor_info=metadata, timeout=1.0)) as scans:
        with pytest.raises(sensor.ClientTimeout):
            next(iter(scans))


def test_longform_client(hil_sensor_hostname, lidar_mode, timestamp_mode, lidar_port, imu_port) -> None:
    cli = _client.SensorConnection(hil_sensor_hostname, "", lidar_mode, timestamp_mode, lidar_port, imu_port, 60)
