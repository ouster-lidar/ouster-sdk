"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.
"""

from pathlib import Path
from click.testing import CliRunner
from ouster.cli import core
from ouster.cli.core.cli_args import CliArgs
from ouster.cli.plugins import source, source_osf  # noqa: F401

import os
import pytest

from ouster.sdk import open_source
from ouster.sdk.client import LidarPacket, ImuPacket
from tests.conftest import BAGS_DATA_DIR

from ouster.sdk.bag import BagPacketSource, BagScanSource


class set_directory(object):
    """Sets the cwd within the context

    Args:
      path (Path): The path to the cwd
    """
    def __init__(self, path: Path):
        self.path = path
        self.origin = Path().absolute()

    def __enter__(self):
        os.chdir(self.path)

    def __exit__(self, exc_type, exc_value, traceback):
        os.chdir(self.origin)


@pytest.fixture
def test_bag_file() -> str:
    return str(Path(BAGS_DATA_DIR) / '512x10_raw.bag')


def test_bag_convert(tmp_path, test_bag_file):
    """Read in a raw rosbag and convert to ros2 and make sure we got all messages"""
    with set_directory(tmp_path):
        assert not os.listdir(tmp_path)  # no files in output dir
        result = CliRunner().invoke(core.cli, CliArgs(['source', test_bag_file, 'save_raw', '--ros2', 'test.bag']).args)
        print(result.output)
        assert result.exit_code == 0
        # there's at most one OSF file in output dir
        files = os.listdir(tmp_path)
        assert len(files) == 1

        # now verify it has all messages
        src = BagScanSource("test.bag")

        c = 0
        for s in src:
            assert len(s[0].fields) == 4
            c += 1
        assert c == 1

        src.close()

        psource = BagPacketSource("test.bag")
        lp = 0
        ip = 0
        for idx, p in psource:
            if isinstance(p, LidarPacket):
                lp += 1
            if isinstance(p, ImuPacket):
                ip += 1
        assert lp == 32
        assert ip == 10


def test_bag_save(test_bag_file, tmp_path):
    with set_directory(tmp_path):
        assert not os.listdir(tmp_path)  # no files in output dir
        result = CliRunner().invoke(core.cli, CliArgs(['source', test_bag_file, 'save_raw', 'test.bag']).args)
        print(result.output)
        assert result.exit_code == 0
        # there's at most one OSF file in output dir
        files = os.listdir(tmp_path)
        assert len(files) == 1

        # now verify it has all messages
        src = BagScanSource("test.bag")

        c = 0
        for s in src:
            assert len(s[0].fields) == 4
            c += 1
        assert c == 1

        src.close()

        psource = BagPacketSource("test.bag")
        lp = 0
        ip = 0
        for idx, p in psource:
            if isinstance(p, LidarPacket):
                lp += 1
            if isinstance(p, ImuPacket):
                ip += 1
        assert lp == 32
        assert ip == 10


def test_bag_open(test_bag_file):
    # verify it has all messages
    src = open_source(test_bag_file)

    c = 0
    for s in src:
        assert len(s.fields) == 4
        c += 1
    assert c == 1

    src.close()

    src = BagScanSource(test_bag_file)

    c = 0
    for s in src:
        assert len(s[0].fields) == 4
        c += 1
    assert c == 1

    src.close()

    psource = BagPacketSource(test_bag_file)
    lp = 0
    ip = 0
    for idx, p in psource:
        if isinstance(p, LidarPacket):
            lp += 1
        if isinstance(p, ImuPacket):
            ip += 1
    assert lp == 32
    assert ip == 10


def test_bag_info(tmp_path, test_bag_file):
    with set_directory(tmp_path):
        assert not os.listdir(tmp_path)  # no files in output dir
        result = CliRunner().invoke(core.cli, CliArgs(['source', test_bag_file, 'info']).args)
        print(result.output)
        assert result.exit_code == 0
        assert "Message Count: 43" in result.output


def test_bag_metadata(tmp_path, test_bag_file):
    with set_directory(tmp_path):
        assert not os.listdir(tmp_path)  # no files in output dir
        result = CliRunner().invoke(core.cli, CliArgs(['source', test_bag_file, 'metadata']).args)
        print(result.output)
        assert result.exit_code == 0
        assert "prod_sn" in result.output
        assert "initialization_id" in result.output
        assert "imu_intrinsics" in result.output
