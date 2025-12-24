import pytest
import tempfile
import numpy as np
from ouster.sdk.core import SensorInfo
from ouster.sdk.zone_monitor import Zrb


def test_client_rendered_zone_write_checks():
    with pytest.raises(RuntimeError, match="Zrb::save: serial number not set"):
        zrb = Zrb()
        zrb.near_range_mm = np.zeros((4, 4), dtype=np.uint16)
        zrb.far_range_mm = np.zeros((4, 4), dtype=np.uint16)
        zrb.blob()
    with pytest.raises(RuntimeError, match="Zrb::save: near image data missing"):
        zrb = Zrb()
        zrb.serial_number = 122247000785
        zrb.far_range_mm = np.zeros((4, 4), dtype=np.uint16)
        zrb.blob()


def test_client_rendered_zone_write_valid(test_data_dir):
    sensor_info = SensorInfo(open(f'{test_data_dir}/zone_monitor/785.json').read())
    max_error = 4  # in mm
    zrb = Zrb()
    zrb.serial_number = 122247000785
    zrb.near_range_mm = np.zeros((4, 4), dtype=np.uint32)
    zrb.near_range_mm[1, 1] = 1000
    zrb.near_range_mm[2, 2] = 2000
    zrb.far_range_mm = np.zeros((4, 4), dtype=np.uint32)
    zrb.far_range_mm[1, 1] = 3000
    zrb.far_range_mm[2, 2] = 3000
    zrb.beam_to_lidar_transform = sensor_info.beam_to_lidar_transform
    zrb.lidar_to_sensor_transform = sensor_info.lidar_to_sensor_transform
    zrb.sensor_to_body_transform = np.eye(4)
    blob = zrb.blob()
    zrb2 = Zrb(blob)
    assert np.allclose(zrb2.near_range_mm, zrb.near_range_mm, atol=max_error)
    assert np.allclose(zrb2.far_range_mm, zrb.far_range_mm, atol=max_error)
    assert zrb.stl_hash is None
    assert zrb2.stl_hash is None
    assert zrb.stl_hash == zrb2.stl_hash
    assert zrb.serial_number == zrb2.serial_number
    assert zrb.hash == zrb2.hash
    assert zrb.blob() == zrb2.blob()
    try:
        with tempfile.NamedTemporaryFile(delete=False) as f:
            zrb.save(f.name)
            f.close()
        zrb3 = Zrb(f.name)
        assert zrb3.blob() == zrb.blob()
        # Important! because encoding is lossy,
        # we don't expect a Zrb loaded from a blob to be exactly equal to the original
        # assert zrb == zrb2
    finally:
        import os
        try:
            os.remove(f.name)
        except OSError:
            pass


def test_fail_zrb_zoneset_zrb_has_no_data(test_data_dir):
    """It should fail if a zrb has no data."""
    sensor_info = SensorInfo(open(f'{test_data_dir}/pcaps/OS-0-128_v3.0.1_1024x10.2.json').read())
    zrb = Zrb()
    zrb.serial_number = sensor_info.sn
    with pytest.raises(RuntimeError, match="Zrb::save: near image data missing"):
        zrb.blob()
