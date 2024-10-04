"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from copy import copy

import json
import numpy
import pytest
import inspect
from pathlib import Path

from ouster.sdk import client

from tests.conftest import METADATA_DATA_DIR


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


def test_lidar_mode_misc() -> None:
    """Check some misc properties of lidar mode."""
    assert len(
        client.LidarMode.__members__) == 7, "Don't forget to update tests!"
    assert client.LidarMode.from_string('foo') == client.LidarMode.MODE_UNSPEC
    assert client.LidarMode(0) == client.LidarMode.MODE_UNSPEC
    assert str(client.LidarMode.MODE_UNSPEC) == "UNKNOWN"


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_read_info(meta: client.SensorInfo) -> None:
    """Check the particular values in the test data."""
    assert meta.sn == "992029000352"
    assert meta.fw_rev == "v2.0.0-rc.2"
    assert meta.config.lidar_mode == client.LidarMode.MODE_1024x20
    assert meta.prod_line == "OS-2-32-U0"
    assert meta.format.columns_per_frame == 1024
    assert meta.format.columns_per_packet == 16
    assert meta.w == meta.format.columns_per_frame
    assert meta.h == meta.format.pixels_per_column
    assert meta.format.column_window[0] == 0
    assert meta.format.column_window[1] == meta.format.columns_per_frame - 1
    assert len(meta.format.pixel_shift_by_row) == 32
    assert meta.format.pixels_per_column == 32
    assert len(meta.beam_azimuth_angles) == 32
    assert len(meta.beam_altitude_angles) == 32
    assert meta.format.udp_profile_lidar == client.UDPProfileLidar.PROFILE_LIDAR_LEGACY
    assert meta.format.udp_profile_imu == client.UDPProfileIMU.PROFILE_IMU_LEGACY
    assert meta.imu_to_sensor_transform.shape == (4, 4)
    assert meta.lidar_to_sensor_transform.shape == (4, 4)
    assert meta.lidar_origin_to_beam_origin_mm == 13.762

    beam_to_lidar_transform = numpy.identity(4)
    beam_to_lidar_transform[0, 3] = meta.lidar_origin_to_beam_origin_mm
    assert numpy.array_equal(meta.beam_to_lidar_transform, beam_to_lidar_transform)

    assert numpy.array_equal(meta.extrinsic, numpy.identity(4))
    assert meta.init_id == 0
    assert meta.config.udp_port_lidar is None
    assert meta.config.udp_port_imu is None

    assert meta.build_date == "2020-10-23T14:05:18Z"
    assert meta.prod_pn == "840-102146-C"
    assert meta.image_rev == "ousteros-image-prod-aries-v2.0.0-rc.2+20201023140416.staging"
    assert meta.status == "RUNNING"
    assert meta.cal == client.SensorCalibration()

    ref_config = client.SensorConfig()
    ref_config.lidar_mode = client.LidarMode.MODE_1024x20
    assert meta.config == ref_config
    assert meta.get_version().major == 2

    product_info = meta.get_product_info()
    assert product_info.full_product_info == meta.prod_line
    assert product_info.form_factor == "OS2"
    assert product_info.short_range is False
    assert product_info.beam_config == "U0"
    assert product_info.beam_count == 32


def test_write_info(meta: client.SensorInfo) -> None:
    """Check modifying metadata."""
    meta.sn = ""
    meta.fw_rev = ""
    meta.prod_line = ""
    meta.format.columns_per_frame = 0
    meta.format.columns_per_packet = 0
    meta.format.pixels_per_column = 0
    meta.format.column_window = (0, 0)
    meta.format.pixel_shift_by_row = []
    meta.format.udp_profile_lidar = client.UDPProfileLidar(0)
    meta.format.udp_profile_imu = client.UDPProfileIMU(0)
    meta.format.fps = 0
    meta.beam_azimuth_angles = []
    meta.beam_altitude_angles = []
    meta.imu_to_sensor_transform = numpy.zeros((4, 4))
    meta.lidar_to_sensor_transform = numpy.zeros((4, 4))
    meta.extrinsic = numpy.zeros((4, 4))
    meta.lidar_origin_to_beam_origin_mm = 0.0
    meta.beam_to_lidar_transform = numpy.zeros((4, 4))
    meta.init_id = 0
    meta.build_date = ""
    meta.image_rev = ""
    meta.prod_pn = ""
    meta.status = ""
    meta.user_data = ""
    meta.cal = client.SensorCalibration()
    meta.config = client.SensorConfig()
    meta.config.udp_port_lidar = None
    meta.config.udp_port_imu = None
    meta.config.lidar_mode = None

    assert meta == client.SensorInfo()
    assert meta.has_fields_equal(client.SensorInfo())
    client.SensorInfo().to_json_string()
    assert meta.to_json_string() == client.SensorInfo().to_json_string()

    with pytest.raises(TypeError):
        meta.mode = 1  # type: ignore
    with pytest.raises(TypeError):
        meta.imu_to_sensor_transform = numpy.zeros((4, 5))
    with pytest.raises(TypeError):
        meta.lidar_to_sensor_transform = numpy.zeros((3, 4))
    with pytest.raises(TypeError):
        meta.extrinsic = numpy.zeros(16)
    with pytest.raises(TypeError):
        meta.beam_altitude_angles = 1  # type: ignore
    with pytest.raises(TypeError):
        meta.beam_azimuth_angles = ["foo"]  # type: ignore

    product_info = meta.get_product_info()
    with pytest.raises(AttributeError):
        product_info.full_product_info = ""
    with pytest.raises(AttributeError):
        product_info.form_factor = ""
    with pytest.raises(AttributeError):
        product_info.short_range = True
    with pytest.raises(AttributeError):
        product_info.beam_config = ""
    with pytest.raises(AttributeError):
        product_info.beam_count = 1


def test_copy_info(meta: client.SensorInfo) -> None:
    """Check that copy() works."""
    meta1 = copy(meta)

    assert meta1 is not meta
    assert meta1 == meta

    meta1.format.columns_per_packet = 42
    assert meta1 != meta

    meta2 = copy(meta)
    meta2.sn = "foo"
    assert meta2 != meta


def test_userdata(meta: client.SensorInfo) -> None:
    meta.user_data = "test"

    meta2 = client.SensorInfo(meta.to_json_string(), True)

    assert meta2.user_data == meta.user_data


def test_extrinsics(meta: client.SensorInfo) -> None:
    copy = numpy.eye(4)
    copy[0, 0] = 5
    copy[1, 0] = 3
    meta.extrinsic = copy

    meta2 = client.SensorInfo(meta.to_json_string(), True)

    assert (meta2.extrinsic == meta.extrinsic).all()


def test_parse_info() -> None:
    """Sanity check parsing from json."""
    with pytest.raises(RuntimeError):
        client.SensorInfo('/')
    with pytest.raises(RuntimeError):
        client.SensorInfo('')
    with pytest.raises(RuntimeError):
        client.SensorInfo('{  }')
    with pytest.raises(RuntimeError):
        client.SensorInfo('{ "lidar_mode": "1024x10" }')

    # TODO: this should actually fail unless *all* parameters needed to
    # unambiguously interpret a sensor data stream are present
    metadata = {
        'lidar_mode': '1024x10',
        'beam_altitude_angles': [1] * 64,
        'beam_azimuth_angles': [1] * 64,
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


def test_info_length() -> None:
    """Check length of info to ensure we've added appropriately to the == operator"""

    info_attributes = inspect.getmembers(client.SensorInfo, lambda a: not inspect.isroutine(a))
    info_properties = [a for a in info_attributes if not (a[0].startswith('__') and a[0].endswith('__'))]

    assert len(info_properties) == 24, "Don't forget to update tests and the sensor_info == operator!"


def test_equality_format() -> None:
    """Check length of data format to ensure we've added appropriately to the == operator"""

    data_format_attributes = inspect.getmembers(client.DataFormat, lambda a: not inspect.isroutine(a))
    data_format_properties = [a for a in data_format_attributes if not (a[0].startswith('__') and a[0].endswith('__'))]

    assert len(data_format_properties) == 8, "Don't forget to update tests and the data_format == operator!"


def test_skip_metadata_beam_validation() -> None:
    """Check that skipping beam validation works"""

    all_zeros_metadata = str(Path(METADATA_DATA_DIR) / "malformed/complete_but_all_zeros_legacy.json")

    # test that you can simply initialize without any metadata specified
    client.SensorInfo()

    with open(all_zeros_metadata, 'r') as f:
        # test that specifying skip doesn't raise error
        client.SensorInfo(f.read())


@pytest.mark.parametrize('metadata_key', [
    '1_12',
    '1_12_legacy',
    '1_13',
    '1_14_128_legacy',
    '2_0',
    '2_0_legacy',
    '2_1',
    '2_1_legacy', '2_2', '2_2_legacy', '2_3',
    '2_3_legacy', '2_4', '2_4_legacy', '2_5', '2_5_legacy', '3_0'])
def test_updated_string(metadata_base_name) -> None:
    meta_path = str(Path(METADATA_DATA_DIR) / f"{metadata_base_name}")
    original_string = ""
    with open(meta_path, 'r') as f:
        original_string = f.read()
        meta = client.SensorInfo(original_string)

    meta.format.columns_per_packet = 40
    meta.format.fps = 50  # crucial check - fps is a value that is filled in with a default value at parsing time
    updated_metadata_string = meta.to_json_string()

    assert updated_metadata_string != original_string

    meta2 = client.SensorInfo(updated_metadata_string, skip_beam_validation=True)
    assert meta2.format.columns_per_packet == 40
    assert meta2.format.fps == 50
