"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from contextlib import closing
from os import path
from typing import Iterator
from pathlib import Path

from more_itertools import partition
import pytest

from ouster import client, pcap

pytest.register_assert_rewrite('ouster.client._digest')
import ouster.client._digest as digest  # noqa


_has_mapping = False
try:
    from ouster.cli.plugins import cli_mapping  # type: ignore # noqa: F401 # yes... it has to be in this order.
    _has_mapping = True
except ImportError:
    pass


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
# TODO: add OS-DOME-32/64 in 1024x10 mode pcap with digest
PCAPS_DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "../../tests/pcaps")
METADATA_DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "../../tests/metadata")
OSFS_DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "../../tests/osfs")

TESTS = {
    'legacy-2.0': 'OS-2-32-U0_v2.0.0_1024x10',
    'legacy-2.1': 'OS-1-32-G_v2.1.1_1024x10',
    'dual-2.2': 'OS-0-32-U1_v2.2.0_1024x10',
    'single-2.3': 'OS-2-128-U1_v2.3.0_1024x10',
    'low-data-rate-2.3': 'OS-0-128-U1_v2.3.0_1024x10',
}


@pytest.fixture(scope='module', params=TESTS.keys())
def test_key(request) -> str:
    return request.param


@pytest.fixture
def base_name(test_key: str) -> str:
    return TESTS[test_key]


@pytest.fixture
def stream_digest(base_name: str):
    digest_path = path.join(PCAPS_DATA_DIR, f"{base_name}_digest.json")
    with open(digest_path, 'r') as f:
        return digest.StreamDigest.from_json(f.read())


@pytest.fixture
def meta(base_name: str):
    meta_path = path.join(PCAPS_DATA_DIR, f"{base_name}.json")
    with open(meta_path, 'r') as f:
        return client.SensorInfo(f.read())


@pytest.fixture
def meta_2_0():
    meta_path = path.join(PCAPS_DATA_DIR, f"{TESTS['legacy-2.0']}.json")
    with open(meta_path, 'r') as f:
        return client.SensorInfo(f.read())


@pytest.fixture
def real_pcap_path(base_name: str, meta: client.SensorInfo) -> str:
    return path.join(PCAPS_DATA_DIR, f"{base_name}.pcap")


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


@pytest.fixture(scope="package")
def test_data_dir():
    return Path(path.dirname(path.abspath(__file__))) / ".." / ".." / "tests"


METADATAS = {
        '1_12': '1_12_os1-991913000010-64.json',
        '1_12_legacy': '1_12_os1-991937000062-64_legacy.json',
        '1_13': '1_13_os1-991913000010-64.json',
        '1_13_legacy': '1_13_os1-991937000062-32A02_legacy.json',
        '1_14_128_legacy': '1_14_6cccd_os-882002000138-128_legacy.json',
        '2_0': '2_0_0_os1-991913000010-64.json',
        '2_0_legacy': '2_0_0_os1-992008000494-128_col_win_legacy.json',
        '2_1': '2_1_2_os1-991913000010-64.json',
        '2_1_legacy': '2_1_2_os1-991913000010-64_legacy.json',
        '2_2': '2_2_os-992119000444-128.json',
        '2_2_legacy': '2_2_os-992119000444-128_legacy.json',
        '2_3': '2_3_1_os-992146000760-128.json',
        '2_3_legacy': '2_3_1_os-992146000760-128_legacy.json',
        '2_4': '2_4_0_os-992146000760-128.json',
        '2_4_legacy': '2_4_0_os-992146000760-128_legacy.json',
        '2_5': '2_5_0_os-992146000760-128.json',
        '2_5_legacy': '2_5_0_os-992146000760-128_legacy.json',
        '3_0': '3_0_1_os-122246000293-128.json',
        '3_0_legacy': '3_0_1_os-122246000293-128_legacy.json',
        'ouster-studio-reduced': 'ouster-studio-reduced-config-v1.json',
}


@pytest.fixture(scope='module', params=METADATAS.keys())
def metadata_key(request) -> str:
    return request.param


@pytest.fixture
def metadata_base_name(metadata_key: str) -> str:
    return METADATAS[metadata_key]


@pytest.fixture
def has_mapping() -> bool:
    return _has_mapping
