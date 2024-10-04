import pytest
import numpy as np

import ouster.sdk.osf as osf
from ouster.sdk import open_source


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


@pytest.fixture
def sensor_metadata(input_osf_file):
    """Fixture to just get a valid sensor metadata."""
    reader = osf.Reader(str(input_osf_file))
    lidar_meta = reader.meta_store.get(osf.LidarSensor)
    return lidar_meta.metadata


def test_osf_save_extrinsics(tmp_path, sensor_metadata):
    output_osf_file = tmp_path / "out.osf"

    writer = osf.Writer(str(output_osf_file))
    lidar_id = writer.add_metadata(osf.LidarSensor(sensor_metadata))

    ext_mat = np.eye(4)
    # some translation
    ext_mat[0, 3] = 10
    # some rotation
    ext_mat[1, 1] = ext_mat[2, 2] = np.cos(np.pi / 8)
    ext_mat[2, 1] = np.sin(np.pi / 8)
    ext_mat[1, 2] = -ext_mat[2, 1]

    writer.add_metadata(osf.Extrinsics(ext_mat, lidar_id, "test_calibrated"))

    writer.close()

    # Read extrinsic back and compare
    reader = osf.Reader(str(output_osf_file))
    extrinsics_meta = reader.meta_store.get(osf.Extrinsics)

    assert (ext_mat == extrinsics_meta.extrinsics).all()
    assert lidar_id == extrinsics_meta.ref_meta_id
    assert "test_calibrated" == extrinsics_meta.name


def test_osf_read_old_extrinsics(tmp_path, sensor_metadata):
    """OsfScanSource's SensorInfo (from the metadata attribute)
    should include extrinsics loaded from the osf metadata store."""

    # write an OSF with some extrinsics
    output_osf_file = tmp_path / "out.osf"

    writer = osf.Writer(str(output_osf_file))
    lidar_id = writer.add_metadata(osf.LidarSensor(sensor_metadata))

    ext_mat = np.eye(4)
    # some translation
    ext_mat[0, 3] = 10
    # some rotation
    ext_mat[1, 1] = ext_mat[2, 2] = np.cos(np.pi / 8)
    ext_mat[2, 1] = np.sin(np.pi / 8)
    ext_mat[1, 2] = -ext_mat[2, 1]

    writer.add_metadata(osf.Extrinsics(ext_mat, lidar_id, "test_calibrated"))
    writer.close()

    # get an OsfScanSource from open_source
    src = open_source(str(output_osf_file), sensor_idx=-1)
    assert (ext_mat == src.metadata[0].extrinsic).all()
