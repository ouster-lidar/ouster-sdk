"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from contextlib import closing
from os import path, environ
from typing import Iterator
from pathlib import Path

import click.testing as _ct
from more_itertools import partition
import pytest
from ouster.sdk import core, pcap
from ouster.sdk.viz import Cloud

# Workaround for Click CliRunner bug (pallets/click#824, fixed in Click 8.3.2
# via PR #3139). _NamedTextIOWrapper.close() closes the BytesIO buffer it
# wraps, causing "ValueError: I/O operation on closed file" when background
# threads outlive cli.main(). Remove this once Click >= 8.3.2 is available.
_ct._NamedTextIOWrapper.close = lambda self: None  # type: ignore[assignment]

pytest.register_assert_rewrite('ouster.sdk.core._digest')
import ouster.sdk.core._digest as digest  # noqa

_has_mapping = False
try:
    from ouster.cli.plugins import cli_mapping  # type: ignore # noqa: F401 # yes... it has to be in this order.

    _has_mapping = False  # NOTE: temporarily disabled due to CLI chaining -- Tim T.
except ImportError:
    pass

_has_perception = False
try:
    from ouster.sdk._bindings.perception import DetectionEngine as _PE
    try:
        # The stub always raises RuntimeError("DetectionEngine is only available
        # with distributed binaries."). A real engine raises TypeError (wrong args)
        # or succeeds, so any non-"distributed binaries" outcome means it's real.
        _PE.create([])
        _has_perception = True
    except RuntimeError as _e:
        _has_perception = "distributed binaries" not in str(_e)
    except Exception:
        _has_perception = True
except ImportError:
    pass


# boilerplate for selecting / deslecting interactive tests
def pytest_addoption(parser):
    parser.addoption("--interactive",
                     action="store_true",
                     required=False,
                     default=False,
                     help="Run interactive tests")
    default_performance = environ.get("OUSTER_PERFORMANCE", "0") == "1"
    parser.addoption("--performance",
                     action="store_true",
                     required=False,
                     default=default_performance,
                     help="""Run longer performance tests.
                     Can also be set using the OUSTER_PERFORMANCE environment variable.""")
    parser.addoption("--num-iterations",
                     required=False,
                     default=0,
                     help="Number of iterations to run for each performance test.")


def pytest_configure(config):
    """Register custom "interactive" marker."""
    config.addinivalue_line("markers", "interactive: run interactive tests")
    config.addinivalue_line("markers", "slow: long-running tests (skipped unless env var ENABLE_SLOW_DOC_TESTS=1)")
    config.addinivalue_line("markers", "performance: perform longer versions of performance tests")
    config.addinivalue_line("markers", "perception: requires a functional DetectionEngine (distributed binaries)")


def pytest_collection_modifyitems(items, config) -> None:
    """Deselect items marked 'interactive' or 'slow' unless the corresponding flag is set."""
    all_deselected: list = []

    non_interactive_it, interactive_it = partition(
        lambda item: bool(item.get_closest_marker("interactive")), items)
    non_interactive = list(non_interactive_it)
    interactive = list(interactive_it)
    if config.option.interactive:
        selected = interactive
        all_deselected.extend(non_interactive)
    else:
        selected = non_interactive
        all_deselected.extend(interactive)

    slow_enabled = environ.get("ENABLE_SLOW_DOC_TESTS", "0") == "1"
    non_slow_it, slow_it = partition(
        lambda item: bool(item.get_closest_marker("slow")), selected)
    non_slow = list(non_slow_it)
    slow = list(slow_it)
    if slow_enabled:
        selected = slow
        all_deselected.extend(non_slow)
    else:
        selected = non_slow
        all_deselected.extend(slow)

    # Skip tests marked 'perception' when the real DetectionEngine is unavailable
    # (i.e. the build only contains the stub that raises RuntimeError on create()).
    if not _has_perception:
        non_perception_it, perception_it = partition(
            lambda item: bool(item.get_closest_marker("perception")), selected)
        non_perception = list(non_perception_it)
        perception = list(perception_it)
        for item in perception:
            item.add_marker(pytest.mark.skip(reason="DetectionEngine requires distributed binaries"))
        selected = non_perception + perception

    config.hook.pytest_deselected(items=all_deselected)
    items[:] = selected


# test data
# TODO: add OS-DOME-32/64 in 1024x10 mode pcap with digest
CURRENT_DIR = path.dirname(path.abspath(__file__))
PCAPS_DATA_DIR = path.join(CURRENT_DIR, "..", "..", "tests", "pcaps")
BAGS_DATA_DIR = path.join(CURRENT_DIR, "..", "..", "tests", "bags")
METADATA_DATA_DIR = path.join(CURRENT_DIR, "..", "..", "tests", "metadata")
OSFS_DATA_DIR = path.join(CURRENT_DIR, "..", "..", "tests", "osfs")

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
        return core.SensorInfo(f.read())


@pytest.fixture
def meta_2_0():
    meta_path = path.join(PCAPS_DATA_DIR, f"{TESTS['legacy-2.0']}.json")
    with open(meta_path, 'r') as f:
        return core.SensorInfo(f.read())


@pytest.fixture
def real_pcap_path(base_name: str, meta: core.SensorInfo) -> str:
    return path.join(PCAPS_DATA_DIR, f"{base_name}.pcap")


@pytest.fixture
def real_pcap(real_pcap_path: str,
              meta: core.SensorInfo) -> Iterator[pcap.PcapPacketSource]:
    pcap_obj = pcap.PcapPacketSource(real_pcap_path, sensor_info=[meta])
    yield pcap_obj
    pcap_obj.close()


@pytest.fixture
def packet(real_pcap_path: str, meta: core.SensorInfo) -> core.LidarPacket:
    # note: don't want to depend on the pcap fixture, since this consumes the
    # iterator and it can be shared
    with closing(pcap.PcapPacketSource(real_pcap_path, sensor_info=[meta])) as real_pcap:
        for idx, p in real_pcap:
            if isinstance(p, core.LidarPacket):
                return p
        raise RuntimeError("Failed to find lidar packet in test fixture")


@pytest.fixture
def packets(real_pcap_path: str,
            meta: core.SensorInfo) -> core.PacketSource:
    with closing(pcap.PcapPacketSource(real_pcap_path, sensor_info=[meta])) as real_pcap:
        ps = list(real_pcap)
        list2 = []
        for idx, p in ps:
            list2.append(p)
        return core.Packets(list2, meta)


@pytest.fixture
def frame(packets: core.PacketSource) -> core.LidarFrame:
    batcher = core.FrameBatcher(packets.sensor_info[0])
    lidar_frame = core.LidarFrame(packets.sensor_info[0])

    def batch():
        nonlocal lidar_frame
        new_frame = True
        for idx, p in packets:
            new_frame = False
            if isinstance(p, core.LidarPacket) and batcher.batch(p, lidar_frame):
                yield lidar_frame
                lidar_frame = core.LidarFrame(packets.sensor_info[0])
                new_frame = True
        if not new_frame:
            yield lidar_frame
    return next(iter(batch()))


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


@pytest.fixture
def has_perception() -> bool:
    return _has_perception


class MockCamera:
    def dolly(*args, **kwargs):
        pass

    def set_target(*args, **kwargs):
        pass


class MockPointViz():

    class MockTargetDisplay:
        def enable_rings(*args, **kwargs):
            pass

        def set_ring_size(*args, **kwargs):
            pass

        def set_ring_line_width(*args, **kwargs):
            pass

    def __init__(self):
        self.items = set()

    def add(self, cloud: Cloud):
        assert cloud not in self.items
        self.items.add(cloud)

    def remove(self, cloud: Cloud) -> bool:
        if cloud not in self.items:
            return False
        self.items.remove(cloud)
        return True

    def push_key_handler(*args, **kwargs):
        pass

    def add_default_controls(*args, **kwargs):
        pass

    @property
    def target_display(*args, **kwargs):
        return MockPointViz.MockTargetDisplay()

    def set_notification(*args, **kwargs):
        pass

    def update(*args, **kwargs):
        pass

    def run_once(*args, **kwargs):
        pass

    def run(*args, **kwargs):
        pass

    @property
    def camera(self):
        return MockCamera()

    @property
    def viewport_height(self):
        return 100

    def pop_mouse_pos_handler(*args, **kwargs):
        pass

    def pop_mouse_button_handler(*args, **kwargs):
        pass

    def pop_key_handler(*args, **kwargs):
        pass

    def pop_frame_buffer_resize_handler(*args, **kwargs):
        pass
