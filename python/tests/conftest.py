from contextlib import closing
from os import path

import pytest

from ouster import client, pcap

pytest.register_assert_rewrite('ouster.client._digest')
import ouster.client._digest as digest  # noqa

DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "../../tests/pcaps")

TESTS = {
    'legacy-2.0': 'OS-2-32-U0_v2.0.0_1024x10',
    'legacy-2.1': 'OS-1-32-G_v2.1.1_1024x10',
    'dual-2.2': 'OS-0-32-U1_v2.2.0_1024x10',
}


@pytest.fixture(scope='module', params=TESTS.keys())
def test_key(request) -> str:
    return request.param


@pytest.fixture
def base_name(test_key: str) -> str:
    return TESTS[test_key]


@pytest.fixture
def stream_digest(base_name: str):
    digest_path = path.join(DATA_DIR, f"{base_name}_digest.json")
    with open(digest_path, 'r') as f:
        return digest.StreamDigest.from_json(f.read())


@pytest.fixture
def meta(base_name: str):
    meta_path = path.join(DATA_DIR, f"{base_name}.json")
    with open(meta_path, 'r') as f:
        return client.SensorInfo(f.read())


@pytest.fixture
def packet(base_name: str, meta: client.SensorInfo) -> client.LidarPacket:
    pcap_path = path.join(DATA_DIR, f"{base_name}.pcap")
    with closing(pcap.Pcap(pcap_path, meta)) as source:
        for p in source:
            if isinstance(p, client.LidarPacket):
                return p
        raise RuntimeError("Failed to find lidar packet in text fixture")


@pytest.fixture
def packets(base_name: str, meta: client.SensorInfo) -> client.PacketSource:
    pcap_path = path.join(DATA_DIR, f"{base_name}.pcap")
    with closing(pcap.Pcap(pcap_path, meta)) as source:
        ps = list(source)
        return client.Packets(ps, meta)


@pytest.fixture
def scan(packets: client.PacketSource) -> client.LidarScan:
    scans = client.Scans(packets)
    return next(iter(scans))
