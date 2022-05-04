"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from contextlib import closing
from os import path
from typing import Iterator

from more_itertools import partition
import pytest

from ouster import client, pcap

pytest.register_assert_rewrite('ouster.client._digest')
import ouster.client._digest as digest  # noqa


# boilerplate for selecting / deslecting interactive tests
def pytest_addoption(parser):
    parser.addoption("--interactive",
                     action="store_true",
                     required=False,
                     default=False,
                     help="Run interactive tests")


def pytest_configure(config):
    """Register custom "interactive" marker."""
    config.addinivalue_line("markers", "interactive: run interactive tests")


def pytest_collection_modifyitems(items, config) -> None:
    """Deselect any items marked "full" unless the --full flag is set."""

    normal, interactive = partition(
        lambda item: bool(item.get_closest_marker("interactive")), items)

    select, deselect = (interactive,
                        normal) if config.option.interactive else (normal,
                                                                   interactive)

    config.hook.pytest_deselected(items=deselect)
    items[:] = select


# test data
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
def meta_2_0():
    meta_path = path.join(DATA_DIR, f"{TESTS['legacy-2.0']}.json")
    with open(meta_path, 'r') as f:
        return client.SensorInfo(f.read())


@pytest.fixture
def real_pcap_path(base_name: str, meta: client.SensorInfo) -> str:
    return path.join(DATA_DIR, f"{base_name}.pcap")


@pytest.fixture
def real_pcap(real_pcap_path: str,
              meta: client.SensorInfo) -> Iterator[pcap.Pcap]:
    pcap_obj = pcap.Pcap(real_pcap_path, meta)
    yield pcap_obj
    pcap_obj.close()


@pytest.fixture
def packet(real_pcap_path: str, meta: client.SensorInfo) -> client.LidarPacket:
    # note: don't want to depend on the pcap fixture, since this consumes the
    # iterator and it can be shared
    with closing(pcap.Pcap(real_pcap_path, meta)) as real_pcap:
        for p in real_pcap:
            if isinstance(p, client.LidarPacket):
                return p
        raise RuntimeError("Failed to find lidar packet in test fixture")


@pytest.fixture
def packets(real_pcap_path: str,
            meta: client.SensorInfo) -> client.PacketSource:
    with closing(pcap.Pcap(real_pcap_path, meta)) as real_pcap:
        ps = list(real_pcap)
        return client.Packets(ps, meta)


@pytest.fixture
def scan(packets: client.PacketSource) -> client.LidarScan:
    scans = client.Scans(packets)
    return next(iter(scans))
