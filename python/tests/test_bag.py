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
from ouster.sdk import core as sdk_core
from ouster.sdk.core import LidarPacket, ImuPacket
from ouster.sdk.util import frame_to_packets
from tests.conftest import BAGS_DATA_DIR, PCAPS_DATA_DIR

from ouster.sdk.bag import BagPacketSource, BagFrameSetSource


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
        src = BagFrameSetSource("test.bag")

        c = 0
        for s, in src:
            assert len(s.fields) == 4
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
        src = BagFrameSetSource("test.bag")

        c = 0
        for s, in src:
            assert len(s.fields) == 4
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
    for s, in src:
        assert len(s.fields) == 4
        c += 1
    assert c == 1

    src.close()

    src = BagFrameSetSource(test_bag_file)

    c = 0
    for s, in src:
        assert len(s.fields) == 4
        assert s.sensor_info == src.sensor_info[0]
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


def test_bag_sensor_info(tmp_path, test_bag_file):
    with set_directory(tmp_path):
        assert not os.listdir(tmp_path)  # no files in output dir
        result = CliRunner().invoke(core.cli, CliArgs(['source', test_bag_file, 'sensor_info']).args)
        print(result.output)
        assert result.exit_code == 0
        assert "prod_sn" in result.output
        assert "initialization_id" in result.output
        assert "imu_intrinsics" in result.output


def _write_lidar_frames_bag(bag_path: str, info, num_frames: int) -> None:
    """Programmatically write a ROS1 bag containing ``num_frames`` lidar frames.

    Each frame is deparsed into UDP lidar packets (using a distinct frame id so
    the FrameBatcher yields one scan per frame on readback) and written to the
    ``/ouster/lidar_packets`` topic.
    """
    from rosbags.rosbag1 import Writer  # type: ignore
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg  # type: ignore

    packet_type = "ouster_ros/msg/PacketMsg"
    typestore = get_typestore(Stores.ROS1_NOETIC)
    typestore.register(get_types_from_msg("uint8[] buf", packet_type))
    PacketMsg = typestore.types[packet_type]

    # spread packet timestamps out over time at roughly the sensor's frame rate
    dt = int(1e9 / (info.format.fps * info.format.lidar_packets_per_frame()))
    ts = 1

    writer = Writer(bag_path)
    writer.open()
    try:
        conn = writer.add_connection("/ouster/lidar_packets", packet_type, typestore=typestore)
        for frame_id in range(1, num_frames + 1):
            frame = sdk_core.LidarFrame(info)
            frame.frame_id = frame_id
            # mark all columns valid so every packet in the frame is emitted
            frame.status[:] = 0x1
            frame.field(sdk_core.ChanField.RANGE)[:] = 1000

            for packet in frame_to_packets(frame, info):
                if not isinstance(packet, LidarPacket):
                    continue
                packet.host_timestamp = ts
                msg = PacketMsg(packet.buf)
                data = typestore.serialize_ros1(msg, packet_type)
                writer.write(conn, ts, bytes(data))
                ts += dt
    finally:
        writer.close()


def test_bag_to_osf_save(tmp_path):
    """Regression test for a bag-to-OSF save/close deadlock.

    Programmatically builds a ROS1 bag containing 5 lidar frames, saves it as
    OSF through the CLI, then confirms the OSF holds the expected scan count.
    """
    num_frames = 5

    meta_path = str(Path(PCAPS_DATA_DIR) / 'OS-1-32-G_v2.1.1_1024x10.json')
    with open(meta_path, 'r') as f:
        info = sdk_core.LidarInfo(f.read())

    bag_path = str(tmp_path / "frames.bag")
    osf_path = str(tmp_path / "out.osf")

    _write_lidar_frames_bag(bag_path, info, num_frames)
    assert os.path.isfile(bag_path)

    result = CliRunner().invoke(
        core.cli,
        CliArgs(['source', '-m', meta_path, bag_path, 'save', '--overwrite', osf_path]).args)
    print(result.output)
    assert result.exit_code == 0, result.output
    assert os.path.isfile(osf_path)

    src = open_source(osf_path)
    try:
        scan_count = sum(1 for _ in src)
    finally:
        src.close()
    assert scan_count == num_frames
