import pytest
import numpy as np
from ouster.sdk import core
from ouster.sdk.mapping import LIOSlamConfig, SlamConfig, SlamEngine
from ouster.sdk.open_source import open_source


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


def test_slam_update_skips_frame_with_no_valid_columns(input_osf_file):
    """A frame with no valid columns must be skipped, not crash update().

    Regression test for OSDK-1156: flagging every column invalid (zeroed status)
    used to make get_first_valid_column() throw deep in the update path,
    propagating an uncaught RuntimeError out of slam_engine.update().
    """
    source = open_source(str(input_osf_file))
    config = SlamConfig.create("lio")
    config.min_range = 1.0
    config.max_range = 100.0
    config.voxel_size = 0.5  # set so update() doesn't estimate it
    engine = SlamEngine.create(source.sensor_info, config)

    count = 0
    for i, frame_set in enumerate(source):
        if i == 0:
            # flag every column invalid -> zero valid columns
            frame_set[0].status[:] = 0
        engine.update(frame_set)  # must not raise "No valid columns in LidarFrame"
        count += 1

    assert count > 0


def _without_model(info: core.CameraInfo) -> None:
    pass


def _with_zero_dimensions(info: core.CameraInfo) -> None:
    info.model = core.CameraModel(100.0, 100.0, 1.5, 1.0)
    info.format.columns_per_frame = 0
    info.format.pixels_per_column = 0


def _with_zero_focal_length(info: core.CameraInfo) -> None:
    model = core.CameraModel(100.0, 100.0, 1.5, 1.0)
    model.fx = 0.0
    info.model = model


@pytest.mark.parametrize("make_unprojectable",
                         [_without_model, _with_zero_dimensions, _with_zero_focal_length])
def test_slam_update_skips_unprojectable_range_camera(input_osf_file, make_unprojectable):
    """A RANGE-carrying camera whose LUT can't project its frames must be skipped, not raise."""
    source = open_source(str(input_osf_file))
    cam_info = core.CameraInfo()
    cam_info.format.columns_per_frame = 4
    cam_info.format.pixels_per_column = 3
    make_unprojectable(cam_info)
    config = SlamConfig.create("lio")
    config.min_range = 1.0
    config.max_range = 100.0
    config.voxel_size = 0.5
    engine = SlamEngine.create([source.sensor_info[0], cam_info], config)

    count = 0
    for frame_set in source:
        lidar = frame_set[0]
        cam = core.CameraFrame(cam_info)
        cam.w = 4
        cam.h = 3
        cam.add_field("RANGE", np.full((3, 4), 2000, dtype=np.uint32))
        cam.timestamp[0] = int(lidar.timestamp[lidar.get_first_valid_column()])
        engine.update(core.FrameSet([lidar, cam]))
        count += 1

    assert count > 0
    assert engine.get_point_cloud().shape[0] > 0


def test_slam_update_writes_camera_pose(input_osf_file):
    """Camera frames in a FrameSet should receive a pose interpolated from lidar."""
    source = open_source(str(input_osf_file))
    config = SlamConfig.create("lio")
    config.min_range = 1.0
    config.max_range = 100.0
    config.voxel_size = 0.5
    engine = SlamEngine.create(source.sensor_info, config)

    frame_set = next(iter(source))
    lidar = frame_set[0]
    col = lidar.get_first_valid_column()
    cam = core.CameraFrame()
    cam.timestamp[0] = int(lidar.timestamp[col])
    # Sentinel so we can tell slam actually overwrote the camera pose.
    cam.body_to_world[0] = np.diag([2.0, 2.0, 2.0, 1.0])
    frame_set = core.FrameSet(list(frame_set) + [cam])

    engine.update(frame_set)

    np.testing.assert_allclose(
        cam.body_to_world[0], lidar.body_to_world[col], atol=1e-6
    )
    assert not np.allclose(cam.body_to_world[0], np.diag([2.0, 2.0, 2.0, 1.0]))


def test_slam_engine_create_accepts_camera_infos(input_osf_file):
    """SlamEngine.create should accept a mixed lidar + camera sensor list."""
    source = open_source(str(input_osf_file))
    config = SlamConfig.create("lio")
    sensor_infos = list(source.sensor_info) + [core.CameraInfo(), core.CameraInfo()]
    engine = SlamEngine.create(sensor_infos, config)
    assert engine is not None


def test_slam_engine_create_accepts_camera_only():
    """SlamEngine.create should accept a camera-only dataset (no LidarInfo)."""
    config = SlamConfig.create("lio")
    config.voxel_size = 0.5
    engine = SlamEngine.create([core.CameraInfo()], config)
    assert engine is not None


def test_slam_camera_only_update_preserves_camera_pose():
    """Without RANGE, camera-only slam cannot project a cloud and leaves pose as-is."""
    config = SlamConfig.create("lio")
    config.voxel_size = 0.5
    engine = SlamEngine.create([core.CameraInfo()], config)

    cam = core.CameraFrame()
    pose = np.eye(4)
    pose[0, 3] = 5.0
    cam.body_to_world[0] = pose
    engine.update(core.FrameSet([cam]))

    np.testing.assert_allclose(cam.body_to_world[0], pose)


def _camera_beam_angles(w, h, fx, fy, cx, cy):
    """Per-pixel beam angles (deg) + range scaling map, mirroring source_zed."""
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    x_cam = (u - cx) / fx
    y_cam = (v - cy) / fy
    z_cam = np.ones_like(x_cam)
    ox = z_cam
    oy = -x_cam
    oz = -y_cam
    mag = np.sqrt(ox ** 2 + oy ** 2 + oz ** 2)
    az = np.degrees(np.arctan2(oy, ox)).flatten()
    alt = np.degrees(np.arcsin(oz / mag)).flatten()
    return az, alt, mag


def _make_range_camera_info(w=8, h=8):
    cam_info = core.CameraInfo()
    cam_info.model = core.CameraModel(50.0, 50.0, (w - 1) / 2.0, (h - 1) / 2.0)
    cam_info.format.columns_per_frame = w
    cam_info.format.pixels_per_column = h
    # The shared XYZLut derives per-pixel direction/offset from the pinhole
    # intrinsics, so no beam angles are stored on the CameraInfo.
    return cam_info


def test_slam_camera_only_range_builds_map():
    """Camera-only slam should project RANGE and accumulate a local map."""
    w, h = 8, 8
    cam_info = _make_range_camera_info(w, h)
    config = SlamConfig.create("lio")
    config.min_range = 0.1
    config.max_range = 50.0
    config.voxel_size = 0.25
    engine = SlamEngine.create([cam_info], config)

    _, _, mag = _camera_beam_angles(w, h, 50.0, 50.0, (w - 1) / 2.0, (h - 1) / 2.0)
    depth = np.zeros((h, w), dtype=np.float64)
    depth[2:6, 2:6] = 3.0
    depth[3:5, 3:5] = 2.0
    range_mm = np.ascontiguousarray(
        np.where(depth > 0.0, depth * mag * 1000.0, 0.0).astype(np.uint32))
    for i in range(3):
        cam = core.CameraFrame(cam_info)
        cam.w = w
        cam.h = h
        cam.add_field("RANGE", range_mm)
        cam.timestamp[0] = 1000 * (i + 1)
        engine.update(core.FrameSet([cam]))

    cloud = engine.get_point_cloud()
    assert cloud.shape[0] > 0


def _write_camera_only_osf(path):
    from ouster.sdk import osf

    cam_info = core.CameraInfo()
    cam_info.model = core.CameraModel(100.0, 100.0, 0.0, 0.0)

    frame = core.CameraFrame()
    frame.w = 4
    frame.h = 3
    frame.add_field("DEPTH", np.full((3, 4), 2.0, dtype=np.float32))

    with osf.Writer(str(path)) as writer:
        writer.add_sensor(cam_info)
        writer.save(0, frame, 100)
        writer.save(0, frame, 200)


def test_cli_slam_accepts_camera_dataset(tmp_path):
    """ouster-cli source <camera.osf> slam ... should construct SlamEngine with CameraInfo."""
    from click.testing import CliRunner
    from ouster.cli import core as cli_core
    from ouster.cli.plugins import source_mapping  # noqa: F401

    path = tmp_path / "camera_only.osf"
    _write_camera_only_osf(path)

    result = CliRunner().invoke(
        cli_core.cli,
        ["source", str(path), "slam", "--voxel-size", "0.5", "stats"],
    )
    assert result.exit_code == 0, result.output
    assert "No such command" not in result.output
    assert "No sensor info" not in result.output


def test_slam_engine_initialization(input_osf_file):
    """It should initialize without error with a valid LidarInfo and config."""
    source = open_source(str(input_osf_file))
    config = SlamConfig.create("lio")
    config.min_range = 1.0
    config.max_range = 100.0
    # Verification that create method exists and works
    engine = SlamEngine.create(source.sensor_info, config)
    assert engine is not None


def test_slam_engine_default_config(input_osf_file):
    """SlamEngine.create(sensor_info) should use a default LIOSlamConfig."""
    source = open_source(str(input_osf_file))
    engine = SlamEngine.create(source.sensor_info)
    assert engine is not None


def test_slam_config_factory_default_kind():
    """SlamConfig.create() with no args should return an LIOSlamConfig."""
    config = SlamConfig.create()
    assert isinstance(config, LIOSlamConfig)


def test_slam_config_factory_explicit_kind():
    """SlamConfig.create(kind='lio') should return an LIOSlamConfig."""
    config = SlamConfig.create(kind="lio")
    assert isinstance(config, LIOSlamConfig)


def test_slam_config_fields_settable():
    """Fields on a created config should be settable and readable."""
    config = SlamConfig.create()
    config.min_range = 0.5
    config.max_range = 80.0
    assert config.min_range == 0.5
    assert config.max_range == 80.0


def test_slam_config_factory_wrong_type():
    """It should throw when given an unsupported type."""
    with pytest.raises(ValueError, match="Unsupported SlamConfig type"):
        SlamConfig.create("doesntexist")
