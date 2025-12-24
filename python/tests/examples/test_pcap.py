from pathlib import Path
import inspect
import numpy as np

import pytest

from ouster.sdk import core, pcap
from ouster.sdk.examples import pcap as pcap_examples

_REPO_ROOT = Path(__file__).resolve().parents[3]
test_data_root = _REPO_ROOT / "tests" / "pcaps"


def _pcap_path(test_data_dir: Path) -> Path:
    return test_data_root / "OS-2-128-U1_v2.3.0_1024x10.pcap"


def _metadata_path(test_data_dir: Path) -> Path:
    return test_data_root / "OS-2-128-U1_v2.3.0_1024x10.json"


def test_pcap_read_packets_prints_shapes(capsys, test_data_dir: Path):
    pcap_file = _pcap_path(test_data_dir)
    pcap_examples.pcap_read_packets(str(pcap_file), num=0)

    out = capsys.readouterr().out
    assert "encoder counts" in out
    assert "timestamps" in out
    assert "ranges" in out
    assert "(16,)" in out
    assert "(128, 16)" in out
    accel_lines = [line for line in out.splitlines() if "acceleration" in line]
    angular_lines = [line for line in out.splitlines() if "angular_velocity" in line]
    assert accel_lines, "Expected acceleration output"
    assert angular_lines, "Expected angular velocity output"
    accel_vals = [v.strip() for v in accel_lines[0].split("=", 1)[1].split(",")]
    angular_vals = [v.strip() for v in angular_lines[0].split("=", 1)[1].split(",")]
    assert len(accel_vals) >= 3
    assert len(angular_vals) >= 3


def test_pcap_query_scan_lists_fields(capsys, test_data_dir: Path):
    pcap_file = _pcap_path(test_data_dir)
    if not pcap_file.exists():
        pytest.skip("PCAP fixture missing")

    pcap_examples.pcap_query_scan(str(pcap_file))
    out = capsys.readouterr().out
    for name in ["FLAGS", "NEAR_IR", "RANGE", "REFLECTIVITY", "SIGNAL"]:
        assert name in out


def test_pcap_to_ply_writes_expected_points(tmp_path, test_data_dir: Path):
    try:
        import open3d as o3d  # type: ignore
    except ModuleNotFoundError:
        pytest.skip("open3d not available")

    pcap_file = _pcap_path(test_data_dir)
    meta_file = _metadata_path(test_data_dir)

    metadata = core.SensorInfo(meta_file.read_text())

    # limit to first scan from pcap
    scan_source = pcap.PcapScanSource(str(pcap_file), sensor_info=[metadata])
    scan, = next(iter(scan_source))
    assert scan is not None
    expected_xyz = core.XYZLut(metadata)(scan.field(core.ChanField.RANGE))
    expected_points = expected_xyz.reshape(-1, 3)

    ply_dir = tmp_path
    pcap_examples.pcap_to_ply(str(pcap_file), num=1, ply_dir=ply_dir,
                              ply_base="ply_out", ply_ext="ply")

    ply_path = ply_dir / "ply_out_000000.ply"
    if not ply_path.exists():
        pytest.skip("PLY output not produced")

    cloud = o3d.io.read_point_cloud(str(ply_path))  # type: ignore
    points = np.asarray(cloud.points)

    assert points.shape == expected_points.shape
    np.testing.assert_allclose(points[:10], expected_points[:10], rtol=1e-2, atol=1e-2)


def test_pcap_3d_one_scan_returns_xyz_without_visualizing(test_data_dir: Path):
    try:
        import open3d as o3d  # type: ignore # noqa: F401
    except ModuleNotFoundError:
        pytest.skip("open3d not available")
    pcap_file = _pcap_path(test_data_dir)

    xyz = pcap_examples.pcap_3d_one_scan(str(pcap_file), visualize=False)

    assert xyz is not None
    assert xyz.shape[2] == 3


def test_pcap_to_pcd_writes_expected_points(tmp_path, test_data_dir: Path):
    try:
        import open3d as o3d  # type: ignore
    except ModuleNotFoundError:
        pytest.skip("open3d not available")

    pcap_file = _pcap_path(test_data_dir)
    meta_file = _metadata_path(test_data_dir)

    metadata = core.SensorInfo(meta_file.read_text())

    scan_source = pcap.PcapScanSource(str(pcap_file), sensor_info=[metadata])
    scan, = next(iter(scan_source))
    assert scan is not None
    expected_xyz = core.XYZLut(metadata)(scan.field(core.ChanField.RANGE))
    expected_points = expected_xyz.reshape(-1, 3)

    pcap_examples.pcap_to_pcd(str(pcap_file), num=1, pcd_dir=tmp_path,
                              pcd_base="pcd_out", pcd_ext="pcd")

    pcd_path = tmp_path / "pcd_out_000000.pcd"
    if not pcd_path.exists():
        raise RuntimeError("PCD output not produced")

    cloud = o3d.io.read_point_cloud(str(pcd_path))  # type: ignore
    points = np.asarray(cloud.points)

    assert points.shape == expected_points.shape
    assert np.allclose(points[:10], expected_points[:10])


def test_pcap_to_las_writes_expected_points(tmp_path, test_data_dir: Path):
    try:
        import laspy  # type: ignore
    except ModuleNotFoundError:
        pytest.skip("laspy not available")

    pcap_file = _pcap_path(test_data_dir)
    meta_file = _metadata_path(test_data_dir)
    metadata = core.SensorInfo(meta_file.read_text())

    scan_source = pcap.PcapScanSource(str(pcap_file), sensor_info=[metadata])
    scan, = next(iter(scan_source))
    assert scan is not None
    expected_xyz = core.XYZLut(metadata)(scan.field(core.ChanField.RANGE))
    expected_points = expected_xyz.reshape(-1, 3)

    pcap_examples.pcap_to_las(str(pcap_file), num=1, las_dir=tmp_path,
                              las_base="las_out", las_ext="las")

    las_path = tmp_path / "las_out_000000.las"
    if not las_path.exists():
        raise RuntimeError("LAS output not produced")

    las = laspy.read(str(las_path))  # type: ignore
    points = np.vstack([las.x, las.y, las.z]).T

    assert points.shape == expected_points.shape
    np.testing.assert_allclose(points[:10], expected_points[:10], rtol=1e-2, atol=1e-2)


def test_pcap_display_xyz_points_returns_data_without_plot(test_data_dir: Path):
    if "plot" not in inspect.signature(pcap_examples.pcap_display_xyz_points).parameters:
        pytest.skip("pcap_display_xyz_points without plot flag")

    pcap_file = _pcap_path(test_data_dir)
    meta_file = _metadata_path(test_data_dir)
    sensor_info = core.SensorInfo(meta_file.read_text())

    x, y, z, colors = pcap_examples.pcap_display_xyz_points(str(pcap_file),
                                                            plot=False)

    expected_len = sensor_info.format.columns_per_frame * sensor_info.format.pixels_per_column
    assert x.shape == y.shape == z.shape == (expected_len,)
    assert colors.shape == (expected_len,)
    assert colors.dtype == float or np.issubdtype(colors.dtype, np.floating)
    assert colors.min() >= 0 and colors.max() <= 1
