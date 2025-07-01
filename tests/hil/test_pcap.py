from contextlib import closing
from itertools import islice
from more_itertools import take
import logging
from os import path

import numpy as np
import pytest

from ouster.sdk import core, pcap, sensor
from ouster.sdk.core import UDPProfileLidar, PacketFormat, ColHeader

logger = logging.getLogger("HIL")


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


@pytest.fixture
def hil_sensor_config(hil_initial_config, udp_profile_lidar) -> None:
    hil_initial_config.udp_port_lidar = 7502
    hil_initial_config.udp_port_imu = 7503
    hil_initial_config.lidar_mode = core.LidarMode.MODE_2048x10 # will fail on older sensors
    hil_initial_config.udp_profile_lidar = udp_profile_lidar
    hil_initial_config.azimuth_window = (0, 360000)
    return hil_initial_config


def concat_measurement_ids(packets, pf) -> np.ndarray:
    return np.concatenate([
        pf.packet_header(ColHeader.MEASUREMENT_ID, p.buf) for idx, p in packets if isinstance(p, core.LidarPacket)
    ])


def test_pcap_record(hil_sensor_hostname, hil_configured_sensor,
                     tmpdir) -> None:
    """Test that we can record pcaps without dropping packets."""
    n_packets = 640
    pcap_path = path.join(tmpdir, "test.pcap")

    logger.debug(f"Recording to {pcap_path}")
    with closing(sensor.SensorPacketSource(hil_sensor_hostname, timeout=120)) as src:
        metadata = src.sensor_info
        w = metadata[0].format.columns_per_frame
        take(640, src) # flush
        
        def capture_fun(src):
            for idx, p in islice(src, n_packets):
                yield p
        pcap.record(capture_fun(src), pcap_path)

    logger.debug("Done, reading back pcap")
    with closing(pcap.PcapPacketSource(pcap_path, sensor_info=metadata)) as psrc:
        capture = list(psrc)

    logger.debug(f"Read {len(capture)} packets in mode {metadata[0].config.lidar_mode}")
    assert len(capture) == n_packets

    ids = concat_measurement_ids(capture, PacketFormat(metadata[0])).astype(np.int64)
    n_gaps = np.count_nonzero(np.diff(ids) % w != 1)
    logger.debug(f"Measurement id gaps: {n_gaps}")
    assert n_gaps == 0
