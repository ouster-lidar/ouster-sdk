from copy import copy
from os import path

import json
import numpy
import pytest

from ouster import client
import ouster.client._digest as digest

DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "data")


@pytest.mark.parametrize("mode, string", [
    (client.TimestampMode.TIME_FROM_UNSPEC, "UNKNOWN"),
    (client.TimestampMode.TIME_FROM_INTERNAL_OSC, "TIME_FROM_INTERNAL_OSC"),
    (client.TimestampMode.TIME_FROM_SYNC_PULSE_IN, "TIME_FROM_SYNC_PULSE_IN"),
    (client.TimestampMode.TIME_FROM_PTP_1588, "TIME_FROM_PTP_1588"),
])
def test_timestamp_mode(mode, string) -> None:
    """Check timestamp mode (un)parsing."""
    int(mode)  # make sure nothing is raised
    assert str(mode) == string
    assert client.TimestampMode.from_string(string) == mode


def test_timestamp_mode_misc() -> None:
    """Check some misc properties of timestamp modes."""
    assert len(
        client.TimestampMode.__members__) == 4, "Don't forget to update tests!"
    client.TimestampMode.from_string(
        "foo") == client.TimestampMode.TIME_FROM_UNSPEC
    client.TimestampMode(0) == client.TimestampMode.TIME_FROM_UNSPEC


@pytest.mark.parametrize("mode, cols, frequency, string", [
    (client.LidarMode.MODE_512x10, 512, 10, "512x10"),
    (client.LidarMode.MODE_512x20, 512, 20, "512x20"),
    (client.LidarMode.MODE_1024x10, 1024, 10, "1024x10"),
    (client.LidarMode.MODE_1024x20, 1024, 20, "1024x20"),
    (client.LidarMode.MODE_2048x10, 2048, 10, "2048x10"),
])
def test_lidar_mode(mode, cols, frequency, string) -> None:
    """Check lidar mode (un)parsing and cols/frequency."""
    int(mode)  # make sure nothing is raised
    assert str(mode) == string
    assert client.LidarMode.from_string(string) == mode
    assert mode.cols == cols
    assert mode.frequency == frequency


def test_lidar_mode_misc() -> None:
    """Check some misc properties of lidar mode."""
    assert len(
        client.LidarMode.__members__) == 6, "Don't forget to update tests!"
    assert client.LidarMode.from_string('foo') == client.LidarMode.MODE_UNSPEC
    assert client.LidarMode(0) == client.LidarMode.MODE_UNSPEC
    assert str(client.LidarMode.MODE_UNSPEC) == "UNKNOWN"
    with pytest.raises(ValueError):
        client.LidarMode.MODE_UNSPEC.cols
    with pytest.raises(ValueError):
        client.LidarMode.MODE_UNSPEC.frequency
    with pytest.raises(AttributeError):
        client.LidarMode.MODE_1024x10.cols = 1025  # type: ignore
    with pytest.raises(AttributeError):
        client.LidarMode.MODE_1024x10.frequency = 0  # type: ignore


@pytest.fixture()
def metadata() -> client.SensorInfo:
    digest_path = path.join(DATA_DIR, "os-992011000121_digest.json")
    with open(digest_path, 'r') as f:
        return digest.StreamDigest.from_json(f.read()).meta


def test_read_info(metadata: client.SensorInfo) -> None:
    """Check the particular values in the test data."""
    assert metadata.hostname == "os-992011000121"
    assert metadata.sn == "992011000121"
    assert metadata.fw_rev == "v1.14.0-beta.13"
    assert metadata.mode == client.LidarMode.MODE_512x10
    assert metadata.prod_line == "OS-1-32-U0"
    assert metadata.format.columns_per_frame == 512
    assert metadata.format.columns_per_packet == 16
    assert metadata.format.column_window[0] == 0
    assert metadata.format.column_window[1] == metadata.format.columns_per_frame - 1
    assert len(metadata.format.pixel_shift_by_row) == 32
    assert metadata.format.pixels_per_column == 32
    assert len(metadata.beam_azimuth_angles) == 32
    assert len(metadata.beam_altitude_angles) == 32
    assert metadata.imu_to_sensor_transform.shape == (4, 4)
    assert metadata.lidar_to_sensor_transform.shape == (4, 4)
    assert metadata.lidar_origin_to_beam_origin_mm == 15.806
    assert numpy.array_equal(metadata.extrinsic, numpy.identity(4))


def test_write_info(metadata: client.SensorInfo) -> None:
    """Check modifying metadata."""
    metadata.hostname = ""
    metadata.sn = ""
    metadata.fw_rev = ""
    metadata.mode = client.LidarMode.MODE_UNSPEC
    metadata.prod_line = ""
    metadata.format.columns_per_frame = 0
    metadata.format.columns_per_packet = 0
    metadata.format.pixels_per_column = 0
    metadata.format.column_window = (0, 0)
    metadata.format.pixel_shift_by_row = []
    metadata.beam_azimuth_angles = []
    metadata.beam_altitude_angles = []
    metadata.imu_to_sensor_transform = numpy.zeros((4, 4))
    metadata.lidar_to_sensor_transform = numpy.zeros((4, 4))
    metadata.extrinsic = numpy.zeros((4, 4))
    metadata.lidar_origin_to_beam_origin_mm = 0.0

    assert metadata == client.SensorInfo()

    with pytest.raises(TypeError):
        metadata.mode = 1  # type: ignore
    with pytest.raises(TypeError):
        metadata.imu_to_sensor_transform = numpy.zeros((4, 5))
    with pytest.raises(TypeError):
        metadata.lidar_to_sensor_transform = numpy.zeros((3, 4))
    with pytest.raises(TypeError):
        metadata.extrinsic = numpy.zeros(16)
    with pytest.raises(TypeError):
        metadata.beam_altitude_angles = 1  # type: ignore
    with pytest.raises(TypeError):
        metadata.beam_azimuth_angles = ["foo"]  # type: ignore


def test_copy_info(metadata: client.SensorInfo) -> None:
    """Check that copy() works."""
    meta1 = copy(metadata)

    assert meta1 is not metadata
    assert meta1 == metadata

    meta1.format.columns_per_packet = 42
    assert meta1 != metadata

    meta2 = copy(metadata)
    meta2.hostname = "foo"
    assert meta2 != metadata


def test_parse_info() -> None:
    """Sanity check parsing from json."""
    with pytest.raises(ValueError):
        client.SensorInfo('/')
    with pytest.raises(ValueError):
        client.SensorInfo('')
    with pytest.raises(ValueError):
        client.SensorInfo('{  }')
    with pytest.raises(ValueError):
        client.SensorInfo('{ "lidar_mode": "1024x10" }')

    # TODO: this should actually fail unless *all* parameters needed to
    # unambiguously interpret a sensor data stream are present
    metadata = {
        'lidar_mode': '1024x10',
        'beam_altitude_angles': [0] * 64,
        'beam_azimuth_angles': [0] * 64,
        'lidar_to_sensor_transform': list(range(16))
    }
    info = client.SensorInfo(json.dumps(metadata))

    # check that data format defaults are populated
    assert info.format.pixels_per_column == 64
    assert info.format.columns_per_frame == 1024
    assert info.format.columns_per_packet > 0
    assert info.format.column_window[0] == 0
    assert info.format.column_window[1] == 1023
    assert len(info.format.pixel_shift_by_row) == 64

    # the lidar_to_sensor_transform json is interpreted as a 4x4 matrix in
    # row-major order. Numpy also uses row-major storage order.
    assert numpy.array_equal(info.lidar_to_sensor_transform,
                             numpy.array(range(16)).reshape(4, 4))
    assert numpy.array_equal(info.extrinsic, numpy.identity(4))

    metadata['lidar_origin_to_beam_origin_mm'] = 'foo'
    with pytest.raises(RuntimeError):
        client.SensorInfo(json.dumps(metadata))
