# Please note that the tests in `tests/integration` folder cannot be run on your system
# as they use external data (not included in the repository).

from contextlib import closing
import unittest
import os
import socket
import pytest
import glob
import logging
from copy import deepcopy

from ouster.sdk import client, pcap

logger = logging.getLogger(__name__)


pytest.register_assert_rewrite('ouster.client._digest')
from ouster.sdk.client._digest import StreamDigest  # noqa

TEST_DATA_DIR = os.getenv("TEST_DATA_DIR", default="./")

TESTS = []
files = glob.glob(os.path.join(TEST_DATA_DIR, "**/ouster_pyclient_tests.txt"), recursive=True)
for test_list in files:
    directory = os.path.dirname(test_list)
    with open(test_list, 'r') as f:
        for test in f.read().splitlines():
            if "trunc" not in test:
                TESTS.extend([os.path.join(directory, test)])
assert len(TESTS) > 0


def _read_sensor_info(meta_file):
    meta_raw = ""
    if os.path.exists(meta_file):
        with open(meta_file, 'r') as f:
            meta_raw = f.read()
            meta = client.SensorInfo(meta_raw)
    else:
        meta = None

    return meta_raw, meta


def _send_receive_lidar_packets(path, dst_ip, meta):
    """Read a packets from a pcap through a local socket.

    Replay packets one-by-one from a pcap over a local socket and read them in
    using the client bindings.
    """

    # Reserve an ephemeral port by binding to '0'.
    tmp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tmp_socket.bind(('', 0))
    tmp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Use the reserved port for our dummy sensor
    reserved_port = tmp_socket.getsockname()[1]
    dst_lidar_port = reserved_port
    dst_imu_port = dst_lidar_port

    # Create a metadata for the dummy sensor that uses the reserved port
    rcv_meta = client.SensorInfo(meta.to_json_string())
    rcv_meta.config.udp_port_lidar = dst_lidar_port
    rcv_meta.config.udp_port_imu = dst_imu_port
    try:

        packet_counter = 0
        pcap_length = sum(1 for _ in pcap.Pcap(path, meta))

        with closing(
                client.Sensor(dst_ip,
                              None,
                              None,
                              metadata=rcv_meta,
                              timeout=1.0,
                              _flush_before_read=False)) as sensor:
            tmp_socket.close()
            net_packets = iter(sensor)

            # get a handle replay the pcap over the network
            replay2 = pcap._replay(path, meta, dst_ip, dst_lidar_port, dst_imu_port, address=(dst_ip, 0))
            while next(replay2):
                p = deepcopy(next(net_packets))
                yield p
                packet_counter = packet_counter + 1

    except client.ClientError as err:
        logging.info(f"_send_receive error: err = {err}, packet_counter = {packet_counter}, \
            pcap_length = {pcap_length}")
        raise RuntimeError(f"{str(err)}") from None
    finally:
        # just in case
        tmp_socket.close()


def run_receive_data(test_file_prefix):
    dst_ip = socket.gethostbyname(socket.gethostname())
    path = f"{test_file_prefix}.pcap"
    info_raw, info = _read_sensor_info(f"{test_file_prefix}.json")

    # outer_pyclient_test_long does not have a _meta.json
    if info is None:
        info = client.SensorInfo.from_default(
            client.LidarMode.MODE_1024x10)

    it = _send_receive_lidar_packets(path, dst_ip, info)

    with open(f"{test_file_prefix}_digest.json", 'r') as f:
        good = StreamDigest.from_json(f.read())

    other = StreamDigest.from_packets(client.Packets(it, info))
    logger.debug(f"File digest has {len(good.scans)} scans; Other digest has {len(other.scans)} scans")
    assert len(good.scans) > 0

    good.check(other)


def run_read_data(test_file_prefix):
    # We are grabbing the current local address from the system rather than just using 127.0.0.1
    path = f"{test_file_prefix}.pcap"
    info_raw, info = _read_sensor_info(f"{test_file_prefix}.json")
    with open(f"{test_file_prefix}_digest.json", 'r') as f:
        good = StreamDigest.from_json(f.read())
    other = StreamDigest.from_packets(pcap.Pcap(path, info))

    assert len(good.scans) > 0

    good.check(other)


@pytest.mark.parametrize('test', TESTS)
def test_read_data(test):
    run_read_data(test)


@pytest.mark.parametrize('test', TESTS)
def test_receive_data(test):
    run_receive_data(test)


if __name__ == '__main__':
    unittest.main()
