"""
Copyright (c) 2026, Ouster, Inc.
All rights reserved.
"""

import numpy as np
import pytest

from ouster.sdk import core
from ouster.sdk.viz.model import (CameraModel, DEFAULT_CAMERA_RANGE_MM, LidarFrameVizModel,
                                  Palettes)
from ouster.sdk.viz.core import (_frame_set_has_content, _frame_set_is_loop_sentinel,
                                 _get_timestamp)
from .conftest import MockPointViz


def _make_camera_info():
    w, h = 4, 3
    fx = fy = 100.0
    cx = cy = 0.0
    camera_info = core.CameraInfo()
    camera_info.model = core.CameraModel(fx, fy, cx, cy)
    camera_info.format.columns_per_frame = w
    camera_info.format.pixels_per_column = h
    # The shared XYZLut derives per-pixel direction/offset from the pinhole
    # intrinsics, so no beam angles are stored on the CameraInfo.
    return camera_info


def _make_camera_frame(with_imu=False):
    frame = core.CameraFrame()
    frame.w = 4
    frame.h = 3
    # RANGE (mm) with pixel (0,0) at 2 m radial (cx=cy=0 -> unit ray along +X).
    frame.add_field("RANGE", np.full((3, 4), 2000, dtype=np.uint32))
    left = np.zeros((3, 4, 3), dtype=np.uint8)
    left[:, :, 2] = 255
    frame.add_field("LEFT_RGB", left)
    if with_imu:
        # Ouster-format IMU fields, as a ZED camera saved alongside a lidar would
        # carry (see camera_lib.add_imu_fields).
        frame.add_field("IMU_ACC", np.array([[0.0, 0.0, 1.0]], dtype=np.float32),
                        core.FieldClass.FRAME_FIELD)
        frame.add_field("IMU_GYRO", np.zeros((1, 3), dtype=np.float32),
                        core.FieldClass.FRAME_FIELD)
        frame.add_field("IMU_TIMESTAMP", np.array([1000], dtype=np.uint64),
                        core.FieldClass.FRAME_FIELD)
        frame.add_field("IMU_STATUS", np.array([1], dtype=np.uint16),
                        core.FieldClass.FRAME_FIELD)
    return frame


def test_camera_model_projects_range_cloud():
    viz = MockPointViz()
    camera = CameraModel(viz, _make_camera_info(), palettes=Palettes([]))
    frame = _make_camera_frame()
    camera._amend_view_modes(frame)
    camera.update_clouds("RANGE", frame)

    assert camera._w == 4
    assert camera._h == 3
    assert len(camera._clouds) == 1
    assert camera._clouds[0].size == 12
    expected = np.asarray(
        core.XYZLut(_make_camera_info())(frame),
        dtype=np.float32).reshape(-1, 3)
    assert expected.shape == (12, 3)
    # Ouster sensor frame (X forward, Y left, Z up): pixel (0,0) at 2 m -> X=2.
    np.testing.assert_allclose(expected[0], [2.0, 0.0, 0.0], atol=1e-5)


def _make_legacy_camera_info():
    """A DEPTH-era CameraInfo: pinhole only, no width/height or beam angles."""
    camera_info = core.CameraInfo()
    camera_info.model = core.CameraModel(100.0, 100.0, 0.0, 0.0)
    return camera_info


def _make_legacy_camera_frame():
    frame = core.CameraFrame()
    frame.w = 4
    frame.h = 3
    frame.add_field("DEPTH", np.full((3, 4), 2.0, dtype=np.float32))
    return frame


def test_camera_model_legacy_depth_frame_does_not_crash():
    """A legacy DEPTH camera (no beam intrinsics) must not build a LUT or crash.

    Mixed lidar+camera recordings made before the RANGE unification store camera
    frames with only a pinhole model and a DEPTH field. Such cameras can't be
    projected, but viz must still run (showing images and any lidar clouds)
    instead of raising ``ValueError: lut dimensions must be greater than zero``.
    """
    viz = MockPointViz()
    camera = CameraModel(viz, _make_legacy_camera_info(), palettes=Palettes([]))
    frame = _make_legacy_camera_frame()
    camera._amend_view_modes(frame)
    # Must not raise even though the camera cannot be projected.
    camera.update_clouds("RANGE", frame)
    assert camera._xyzlut is None
    assert camera.local_xyz(frame) is None
    assert camera.world_xyz(frame) is None
    # Geometry is still established (for image panels) and the cloud buffer exists.
    assert camera._w == 4 and camera._h == 3
    assert len(camera._clouds) == 1


def test_camera_model_updates_images():
    viz = MockPointViz()
    camera = CameraModel(viz, _make_camera_info(), palettes=Palettes([]))
    frame = _make_camera_frame()
    camera._amend_view_modes(frame)
    camera.update_images(["LEFT_RGB", "RANGE"], frame)
    assert camera._w == 4
    assert camera._h == 3


def test_camera_model_registers_normals_view_mode():
    """NORMALS from the normals CLI must be selectable in camera viz."""
    viz = MockPointViz()
    camera = CameraModel(viz, _make_camera_info(), palettes=Palettes([]))
    frame = _make_camera_frame()
    frame.add_field("NORMALS", np.zeros((3, 4, 3), dtype=np.float32))
    camera._amend_view_modes(frame)

    assert "NORMALS" in camera._image_modes
    assert "NORMALS" in camera._cloud_modes
    camera.update_images(["NORMALS", "LEFT_RGB"], frame)
    camera.update_clouds("NORMALS", frame)


class _RecorderCloud:
    """Minimal stand-in for viz.Cloud that records the key it's given."""
    def __init__(self, size):
        self.size = size
        self.key = None

    def set_range(self, *args, **kwargs):
        pass

    def set_column_poses(self, *args, **kwargs):
        pass

    def set_palette(self, *args, **kwargs):
        pass

    def set_key(self, key):
        self.key = np.asarray(key)


class _RangeRecorderCloud(_RecorderCloud):
    """_RecorderCloud that records the range and column poses it's given."""
    def __init__(self, camera):
        super().__init__(camera._clouds[0].size)
        self._xyzlut = camera._xyzlut
        self.range = None
        self.column_poses = None

    def set_range(self, rng):
        self.range = np.asarray(rng)

    def set_column_poses(self, poses):
        self.column_poses = np.asarray(poses)

    @property
    def xyz(self) -> np.ndarray:
        """(N, 3) world points the GPU draws: each pixel's LUT point, posed by its column."""
        assert self.range is not None and self.column_poses is not None
        local = np.asarray(self._xyzlut(self.range)).reshape(-1, 3)
        poses = self.column_poses[np.arange(local.shape[0]) % self.range.shape[1]]
        return np.einsum("nij,nj->ni", poses[:, :3, :3], local) + poses[:, :3, 3]


def _make_camera_frame_without_range():
    frame = _make_camera_frame()
    frame.del_field("RANGE")
    return frame


def _default_range_cloud(info, frame, **kwargs):
    """The points a CameraModel draws for ``frame`` (which has no RANGE)."""
    camera = CameraModel(MockPointViz(), info, palettes=Palettes([]), **kwargs)
    camera._amend_view_modes(frame)
    camera.update_clouds("LEFT_RGB", frame)
    rec = _RangeRecorderCloud(camera)
    camera._clouds[0] = rec
    camera.update_clouds("LEFT_RGB", frame)
    return camera, rec.xyz


def test_camera_without_range_draws_cloud_at_default_range():
    """A rectified camera frame with no RANGE is drawn on the plane at forward
    distance DEFAULT_CAMERA_RANGE_MM, so its images still appear in 3D, and the
    frame itself is left untouched."""
    frame = _make_camera_frame_without_range()
    camera, xyz = _default_range_cloud(_make_camera_info(), frame)

    default_range_m = DEFAULT_CAMERA_RANGE_MM / 1000.0
    # pixel (0,0) lies on the optical axis (cx=cy=0): straight ahead.
    np.testing.assert_allclose(xyz[0], [default_range_m, 0.0, 0.0], atol=1e-5)
    np.testing.assert_allclose(xyz[:, 0], default_range_m, atol=1e-3)
    assert not frame.has_field("RANGE")
    assert "RANGE" not in camera._image_modes


def test_camera_default_range_is_configurable():
    _, xyz = _default_range_cloud(_make_camera_info(), _make_camera_frame_without_range(),
                                  default_range_mm=2500)
    np.testing.assert_allclose(xyz[:, 0], 2.5, atol=1e-3)


def test_rectified_camera_default_range_is_flush_with_frustum_far_face():
    """With matching defaults, a rectified camera's cloud lies in the plane of
    its frustum's far face, inside the face's outline."""
    w, h = 64, 48
    info = core.CameraInfo()
    info.model = core.CameraModel(40.0, 42.0, 30.5, 25.0)
    info.format.columns_per_frame = w
    info.format.pixels_per_column = h
    info.sensor_to_body = np.eye(4)
    frame = core.CameraFrame()
    frame.w, frame.h = w, h
    frame.add_field("LEFT_RGB", np.zeros((h, w, 3), dtype=np.uint8))
    camera, xyz = _default_range_cloud(info, frame)

    face = camera._frustum_positions(camera._frustum_far)[1:]
    np.testing.assert_allclose(face[:, 0], camera._frustum_far, rtol=1e-6)
    np.testing.assert_allclose(xyz[:, 0], camera._frustum_far, atol=1e-3)
    # Ranges are whole millimeters, so edge pixels may overshoot by a hair.
    tol = 1e-3
    for axis in (1, 2):
        assert np.all(xyz[:, axis] <= face[:, axis].max() + tol)
        assert np.all(xyz[:, axis] >= face[:, axis].min() - tol)


def test_camera_default_range_zero_draws_no_cloud():
    viz = MockPointViz()
    camera = CameraModel(viz, _make_camera_info(), palettes=Palettes([]),
                         default_range_mm=0)
    frame = _make_camera_frame_without_range()
    camera.update_clouds("LEFT_RGB", frame)
    rec = _RangeRecorderCloud(camera)
    camera._clouds[0] = rec
    camera.update_clouds("LEFT_RGB", frame)
    # zero-range pixels aren't drawn
    assert rec.range is not None
    assert np.count_nonzero(rec.range) == 0


def test_camera_default_range_rejects_negative():
    with pytest.raises(ValueError):
        CameraModel(MockPointViz(), _make_camera_info(), palettes=Palettes([]),
                    default_range_mm=-1)


def test_camera_range_field_takes_precedence_over_default_range():
    viz = MockPointViz()
    camera = CameraModel(viz, _make_camera_info(), palettes=Palettes([]))
    frame = _make_camera_frame()  # RANGE = 2000 mm everywhere
    camera.update_clouds("RANGE", frame)
    rec = _RangeRecorderCloud(camera)
    camera._clouds[0] = rec
    camera.update_clouds("RANGE", frame)
    np.testing.assert_allclose(np.linalg.norm(rec.xyz, axis=1), 2.0, atol=1e-5)


def test_camera_cloud_is_structured_and_posed_like_world_xyz():
    """The camera cloud is projected on the GPU from RANGE and the frame pose, and lands
    on the same world points as the CPU projection used by the accumulators."""
    camera = CameraModel(MockPointViz(), _make_camera_info(), palettes=Palettes([]))
    frame = _make_camera_frame()
    pose = np.eye(4)
    pose[:3, :3] = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    pose[:3, 3] = [10.0, -5.0, 2.0]
    frame.body_to_world[0] = pose
    camera.update_clouds("RANGE", frame)
    assert camera._clouds[0].cols == frame.w

    rec = _RangeRecorderCloud(camera)
    camera._clouds[0] = rec
    camera.update_clouds("RANGE", frame)
    np.testing.assert_array_equal(rec.range, frame.field("RANGE"))
    world_xyz = camera.world_xyz(frame)
    assert world_xyz is not None
    np.testing.assert_allclose(rec.xyz, world_xyz, atol=1e-4)

    camera.update_clouds("RANGE", None)
    assert np.count_nonzero(rec.range) == 0


def test_camera_default_range_threads_through_lidar_frame_viz():
    """camera_default_range_mm on LidarFrameViz reaches the CameraModel."""
    from ouster.sdk.viz.core import LidarFrameViz
    viz = MockPointViz()
    lfv = LidarFrameViz([_make_camera_info()], viz, _add_default_controls=False,
                        camera_default_range_mm=750)
    assert lfv._model._cameras[0]._default_range_mm == 750


class _RecorderImage:
    """Minimal stand-in for viz.Image that records the image it's given."""
    def __init__(self):
        self.image = None

    def set_image(self, img):
        self.image = np.asarray(img)

    def set_position(self, *args, **kwargs):
        pass

    def set_hshift(self, *args, **kwargs):
        pass


def test_camera_zeros_cloud_key_when_global_mode_unsupported():
    """In a mixed lidar+camera scene the global cloud mode is a lidar field the
    camera lacks. The camera must zero its cloud key (every point -> palette[0])
    to denote "mode not supported", rather than falling back to another field."""
    viz = MockPointViz()
    camera = CameraModel(viz, _make_camera_info(), palettes=Palettes([]))
    frame = _make_camera_frame()
    camera._amend_view_modes(frame)

    # "REFLECTIVITY" is a lidar-only field; the camera has RANGE/LEFT_RGB.
    assert "REFLECTIVITY" not in camera._cloud_modes

    # Establish geometry/clouds with a supported mode, then swap in a recorder
    # so we can inspect the key the unsupported mode produces.
    camera.update_clouds("RANGE", frame)
    rec = _RecorderCloud(camera._clouds[0].size)
    camera._clouds[0] = rec

    # An unsupported (lidar) mode must zero the key, not color with a fallback.
    camera.update_clouds("REFLECTIVITY", frame)
    assert rec.key is not None
    assert rec.key.shape == (camera._h, camera._w)
    assert np.count_nonzero(rec.key) == 0

    # image panels are also blanked (zeroed) for an unsupported (lidar) field,
    # rather than falling back to a camera-native field.
    rec_img = _RecorderImage()
    camera._images[0] = rec_img
    camera.update_images(["REFLECTIVITY", "LEFT_RGB"], frame)
    assert rec_img.image is not None
    assert np.count_nonzero(rec_img.image) == 0


def test_lidar_frame_viz_model_camera_defaults():
    viz = MockPointViz()
    model = LidarFrameVizModel(viz, [_make_camera_info()])
    frame = _make_camera_frame()
    frames = core.FrameSet([frame])
    model.update(frames)
    assert "LEFT_RGB" in model._image_mode_names[0]
    assert model._cloud_mode_name == "RANGE"


def test_camera_only_frame_set_helpers():
    frame = _make_camera_frame()
    camera_only = core.FrameSet([frame])
    loop_sentinel = core.FrameSet()

    assert len(camera_only) == 1
    assert bool(camera_only) is True
    assert _frame_set_has_content(camera_only)
    assert not _frame_set_is_loop_sentinel(camera_only)
    assert _frame_set_is_loop_sentinel(loop_sentinel)
    assert _get_timestamp(camera_only) == 0


def test_seekable_preserves_camera_only_frames():
    from ouster.sdk.viz.core import _Seekable

    frame = _make_camera_frame()
    camera_only = core.FrameSet([frame])
    seekable = _Seekable(iter([camera_only]))
    out = next(seekable)
    assert len(out.valid_camera_frames()) == 1
    assert out.valid_camera_frames()[0] is not None
    assert out.valid_camera_frames()[0].has_field("LEFT_RGB")


def test_camera_osd_labels():
    from ouster.sdk.viz.core import LidarFrameViz

    viz = MockPointViz()
    camera_info = _make_camera_info()
    camera_info.sn = 39270359
    lfv = LidarFrameViz([camera_info], viz, _add_default_controls=False)
    frame = _make_camera_frame()
    frames = core.FrameSet([frame])
    lfv.update(frames, frame_num=0)
    lfv._update_multi_viz_osd()

    label = lfv._model._cameras[0]._sensor_label.text
    assert "camera [CTRL+1]" in label
    assert "39270359" in label
    assert "4x3" in label


def test_osd_labels_omit_shortcut_beyond_max_toggle_keys():
    from ouster.sdk.viz.core import LidarFrameViz

    n_cameras = LidarFrameViz.MAX_SENSOR_TOGGLE_KEYS + 1
    viz = MockPointViz()
    lfv = LidarFrameViz([_make_camera_info() for _ in range(n_cameras)], viz,
                        _add_default_controls=False)
    frames = core.FrameSet([_make_camera_frame() for _ in range(n_cameras)])
    lfv.update(frames, frame_num=0)
    lfv._update_multi_viz_osd()

    labels = [camera._sensor_label.text for camera in lfv._model._cameras]
    for i, label in enumerate(labels[:-1]):
        assert f"camera [CTRL+{i + 1}]: " in label
    assert "CTRL+" not in labels[-1]
    assert labels[-1].startswith("camera: ")


def test_camera_image_aspect_ratio_is_derived_from_image():
    """The 2D image layout renders image_w = image_h / _img_aspect_ratio, so
    _img_aspect_ratio must be height/width (as util.img_aspect_ratio is for
    lidar). It is derived from the image's pixel dimensions (not hard-coded);
    a wide image then renders wide, matching the shared row height."""
    viz = MockPointViz()
    camera = CameraModel(viz, _make_camera_info(), palettes=Palettes([]))
    # Unknown until the first image arrives (no hard-coded default).
    assert camera._img_aspect_ratio == 0.0
    frame = _make_camera_frame()  # 4 wide x 3 high
    camera.update_clouds("RANGE", frame)  # triggers _ensure_geometry
    assert camera._img_aspect_ratio == frame.h / frame.w


def test_image_layout_handles_unknown_camera_aspect():
    """The 2D image layout must not divide by zero when a camera's aspect ratio
    is still unknown (no frame seen), and must reflect the image's pixel
    dimensions once a frame arrives."""
    viz = MockPointViz()
    model = LidarFrameVizModel(viz, [_make_camera_info()])
    camera = model._cameras[0]
    # No frame yet: aspect unknown; laying out must not raise.
    model.set_image_size(4)
    assert camera._img_aspect_ratio == 0.0

    frame = _make_camera_frame()  # 4 wide x 3 high -> h/w = 0.75
    model.update(core.FrameSet([frame]))
    model.set_image_size(4)
    assert camera._img_aspect_ratio == frame.h / frame.w


def test_camera_model_detects_imu_data():
    """A camera frame carrying Ouster-format IMU fields flags _has_imu_data,
    just like a lidar SensorModel does."""
    viz = MockPointViz()
    camera = CameraModel(viz, _make_camera_info(), palettes=Palettes([]))
    assert camera._has_imu_data is False
    camera._amend_view_modes(_make_camera_frame(with_imu=True))
    assert camera._has_imu_data is True


def test_camera_without_imu_has_no_imu_data():
    viz = MockPointViz()
    camera = CameraModel(viz, _make_camera_info(), palettes=Palettes([]))
    camera._amend_view_modes(_make_camera_frame(with_imu=False))
    assert camera._has_imu_data is False


def test_camera_skips_view_modes_for_non_image_fields():
    """Only fields matching the CameraInfo's height/width are displayable. The
    per-frame IMU fields are 2D but not image-shaped, so IMU_ACC/IMU_GYRO must
    not be mistaken for RGB images."""
    viz = MockPointViz()
    camera = CameraModel(viz, _make_camera_info(), palettes=Palettes([]))
    camera._amend_view_modes(_make_camera_frame(with_imu=True))

    for name in ("IMU_ACC", "IMU_GYRO", "IMU_TIMESTAMP", "IMU_STATUS"):
        assert name not in camera._image_modes
        assert name not in camera._cloud_modes
    # the correctly-sized image fields are still registered
    assert "LEFT_RGB" in camera._image_modes
    assert "RANGE" in camera._image_modes


def test_camera_skips_view_modes_when_frame_size_differs_from_info():
    """A frame whose images don't match the CameraInfo's h/w yields no view
    modes, rather than modes sized against the wrong geometry."""
    viz = MockPointViz()
    camera = CameraModel(viz, _make_camera_info(), palettes=Palettes([]))
    frame = core.CameraFrame()
    frame.w = 8
    frame.h = 6
    frame.add_field("RANGE", np.full((6, 8), 2000, dtype=np.uint32))
    frame.add_field("LEFT_RGB", np.zeros((6, 8, 3), dtype=np.uint8))
    camera._amend_view_modes(frame)

    assert camera._image_modes == {}
    assert camera._cloud_modes == {}


def test_camera_osd_shows_imu_plot():
    """The OSD shows an IMU plot for a camera carrying IMU data, mirroring the
    lidar behavior."""
    from ouster.sdk.viz.core import LidarFrameViz

    viz = MockPointViz()
    lfv = LidarFrameViz([_make_camera_info()], viz, _add_default_controls=False)
    frame = _make_camera_frame(with_imu=True)
    frames = core.FrameSet([frame])

    # imu_plot() renders the buffered IMU history into the camera's plot without
    # raising; the enumerate index maps to the camera's frame position.
    lfv.update(frames, frame_num=0, last_n_frame_sets=[frames])
    lfv._update_multi_viz_osd()

    camera = lfv._model._cameras[0]
    assert camera._has_imu_data is True
    assert camera._imu_plot.visible is True


def test_camera_osd_hides_imu_plot_without_data():
    """A camera with no IMU data does not get an IMU plot in the OSD."""
    from ouster.sdk.viz.core import LidarFrameViz

    viz = MockPointViz()
    lfv = LidarFrameViz([_make_camera_info()], viz, _add_default_controls=False)
    frame = _make_camera_frame(with_imu=False)
    frames = core.FrameSet([frame])

    lfv.update(frames, frame_num=0, last_n_frame_sets=[frames])
    lfv._update_multi_viz_osd()

    camera = lfv._model._cameras[0]
    assert camera._has_imu_data is False
    assert camera._imu_plot.visible is False


def test_camera_objects_account_for_extrinsics():
    """Objects detected in a camera frame must land in the world using the
    camera's sensor->world transform (frame pose composed with the sensor
    extrinsics), matching the camera point cloud rather than the raw
    object_to_body (which is expressed in the camera sensor frame)."""
    viz = MockPointViz()
    camera_info = _make_camera_info()
    # Non-identity camera extrinsic: mounted 1 m forward and 2 m up.
    sensor_to_body = np.eye(4)
    sensor_to_body[:3, 3] = [1.0, 0.0, 2.0]
    camera_info.sensor_to_body = sensor_to_body

    model = LidarFrameVizModel(viz, [camera_info])

    # Frame (body) pose translated +10 m in X.
    body_to_world = np.eye(4)
    body_to_world[:3, 3] = [10.0, 0.0, 0.0]
    frame = _make_camera_frame()
    frame.body_to_world[0] = body_to_world

    obj = core.Object()
    obj.id = 7
    obj.class_id = 0
    obj.velocity = np.zeros(3)
    obj.dimensions = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    # Object 3 m in front of the camera (camera sensor frame, +X forward).
    obj.object_to_body.position = np.array([3.0, 0.0, 0.0])
    frame.objects['objects'] = [obj]

    model.update_objects(core.FrameSet([frame]))

    camera = model._cameras[0]
    sensor_to_world = camera.camera_to_world(frame)
    # The rendered object (stored on the model) now carries the full sensor->world
    # transform in body_to_world. (frame.objects returns copies, so we inspect the
    # object the model actually rendered.)
    assert len(model._objects) == 1
    rendered_obj = next(iter(model._objects))
    np.testing.assert_allclose(rendered_obj.body_to_world.to_matrix(), sensor_to_world)
    # Cuboid world position: 10 (body) + 1 (extrinsic) + 3 (object) = 14 in X,
    # and 2 (extrinsic) in Z.
    world_pos = (rendered_obj.body_to_world * rendered_obj.object_to_body).position
    np.testing.assert_allclose(world_pos, [14.0, 0.0, 2.0], atol=1e-6)


def test_camera_object_overlay_created_and_populated():
    """A camera with pinhole intrinsics gets a 2D image-panel object overlay,
    and objects are projected onto it with no motion / rolling-shutter
    compensation (the object is used as-is, unlike the lidar dewarp path)."""
    from ouster.sdk.viz.model import ObjectViewMode

    viz = MockPointViz()
    model = LidarFrameVizModel(viz, [_make_camera_info()])
    camera = model._cameras[0]
    # A projectable camera gets an overlay.
    assert camera._object_overlay is not None

    frame = _make_camera_frame()
    obj = core.Object()
    obj.id = 3
    obj.class_id = 0
    obj.velocity = np.zeros(3)
    obj.dimensions = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    obj.object_to_body.position = np.array([3.0, 0.0, 0.0])

    # Camera overlay uses the object as-is (no reference-column dewarp).
    assert camera.overlay_object(obj, frame) is obj

    # With objects present and the default overlay view mode, the camera's
    # overlay must receive one cuboid. Swap in a recorder since ObjectOverlay
    # exposes no cuboid getter.
    frame.objects['objects'] = [obj]
    model.update_objects(core.FrameSet([frame]))
    assert model._object_view_mode == ObjectViewMode.OVERLAY_AND_MAIN_VIEW

    recorded = {}

    class _Recorder:
        def set_cuboids(self, cuboids):
            recorded['cuboids'] = cuboids

    camera._object_overlay = _Recorder()
    model.update_object_cuboids()
    assert len(recorded.get('cuboids', [])) == 1


def test_camera_object_overlay_absent_for_legacy_camera():
    """A legacy camera (no width/height or pinhole intrinsics) can't be
    projected onto a pinhole image, so it gets no object overlay."""
    viz = MockPointViz()
    camera = CameraModel(viz, _make_legacy_camera_info(), palettes=Palettes([]))
    assert camera._object_overlay is None


def _reproject_pinhole(point, model):
    """Project a sensor-frame point back to a pixel via the pinhole model
    (inverse of CameraModel._frustum_positions' back-projection)."""
    x, y, z = point
    return (-y / x * model.fx + model.cx, -z / x * model.fy + model.cy)


def test_camera_frustum_created_and_matches_fov():
    """A projectable camera gets a wireframe frustum whose far corners
    back-project through the pinhole model to the exact image corners."""
    from ouster.sdk.viz.model import DEFAULT_CAMERA_FRUSTUM_FAR
    viz = MockPointViz()
    info = _make_camera_info()
    camera = CameraModel(viz, info, palettes=Palettes([]))

    assert camera._frustum is not None
    assert camera._frustum_far == DEFAULT_CAMERA_FRUSTUM_FAR

    far = DEFAULT_CAMERA_FRUSTUM_FAR
    positions = camera._frustum_positions(far)
    assert positions.shape == (5, 3)
    # apex at the camera origin
    np.testing.assert_allclose(positions[0], [0.0, 0.0, 0.0])
    # all far corners sit on the far plane (forward distance == far)
    np.testing.assert_allclose(positions[1:, 0], far)

    model = info.model
    w, h = info.format.columns_per_frame, info.format.pixels_per_column
    expected_pixels = [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]
    for pos, px in zip(positions[1:], expected_pixels):
        np.testing.assert_allclose(_reproject_pinhole(pos, model), px, atol=1e-3)


def test_camera_frustum_far_is_parameterizable():
    """The far-plane distance scales the frustum and is stored per camera."""
    viz = MockPointViz()
    info = _make_camera_info()
    camera = CameraModel(viz, info, palettes=Palettes([]), frustum_far=7.5)
    assert camera._frustum_far == 7.5
    assert camera._frustum is not None

    near = camera._frustum_positions(2.0)
    far = camera._frustum_positions(6.0)
    # far plane distance follows the parameter, and (cx=cy=0) corners scale
    # linearly with it.
    np.testing.assert_allclose(near[1:, 0], 2.0)
    np.testing.assert_allclose(far[1:, 0], 6.0)
    np.testing.assert_allclose(far[1:], near[1:] * 3.0, atol=1e-5)


def test_camera_frustum_absent_for_legacy_camera():
    """A legacy camera (no pinhole intrinsics / dimensions) gets no frustum."""
    viz = MockPointViz()
    camera = CameraModel(viz, _make_legacy_camera_info(), palettes=Palettes([]))
    assert camera._frustum is None


def test_camera_frustum_added_to_viz_and_toggles():
    """LidarFrameViz adds each camera's frustum to the scene, and toggling the
    stream removes/re-adds it."""
    from ouster.sdk.viz.core import LidarFrameViz
    viz = MockPointViz()
    lfv = LidarFrameViz([_make_camera_info()], viz, _add_default_controls=False)
    frustum = lfv._model._cameras[0]._frustum
    assert frustum is not None
    assert frustum in viz.items

    lfv.toggle_sensor(0, enabled=False)
    assert frustum not in viz.items
    lfv.toggle_sensor(0, enabled=True)
    assert frustum in viz.items


def test_camera_frustum_follows_axis_markers():
    """Hiding/showing axis markers also hides/shows the camera frustums."""
    from ouster.sdk.viz.core import LidarFrameViz
    viz = MockPointViz()
    lfv = LidarFrameViz([_make_camera_info()], viz, _add_default_controls=False)
    frustum = lfv._model._cameras[0]._frustum
    assert frustum is not None
    assert frustum in viz.items

    lfv.toggle_axis_markers(show=False)
    assert frustum not in viz.items
    lfv.toggle_axis_markers(show=True)
    assert frustum in viz.items


def test_camera_frustum_hidden_when_axis_off_and_stream_toggled():
    """With axis markers off, toggling a stream on must not resurrect its
    frustum (frustum visibility is coupled to axis markers)."""
    from ouster.sdk.viz.core import LidarFrameViz
    viz = MockPointViz()
    lfv = LidarFrameViz([_make_camera_info()], viz, _add_default_controls=False)
    frustum = lfv._model._cameras[0]._frustum
    assert frustum is not None

    lfv.toggle_axis_markers(show=False)
    assert frustum not in viz.items
    # Stream off then on again while axis markers are off: frustum stays hidden.
    lfv.toggle_sensor(0, enabled=False)
    lfv.toggle_sensor(0, enabled=True)
    assert frustum not in viz.items
    # Turning axis markers back on restores it (stream is enabled).
    lfv.toggle_axis_markers(show=True)
    assert frustum in viz.items


def test_camera_frustum_far_threads_through_lidar_frame_viz():
    """camera_frustum_far on LidarFrameViz reaches the CameraModel."""
    from ouster.sdk.viz.core import LidarFrameViz
    viz = MockPointViz()
    lfv = LidarFrameViz([_make_camera_info()], viz, _add_default_controls=False,
                        camera_frustum_far=4.25)
    assert lfv._model._cameras[0]._frustum_far == 4.25


def test_camera_axis_follows_camera_pose():
    """Camera-only scene poses should move the camera/base-link axes off the origin."""
    from ouster.sdk.viz.core import LidarFrameViz

    viz = MockPointViz()
    lfv = LidarFrameViz([_make_camera_info()], viz,
                        _add_default_controls=False)

    pose = np.eye(4)
    pose[:3, 3] = [5.0, -3.0, 1.0]
    frame = _make_camera_frame()
    frame.body_to_world[0] = pose
    frames = core.FrameSet([frame])

    lfv.update(frames, frame_num=0)
    lfv._draw_update_scene_poses()

    # sensor_to_body defaults to identity, so the camera axis == body_to_world[0].
    # A single camera is the only stream, so its scene axis is _stream_axis[0].
    np.testing.assert_allclose(lfv._stream_axis[0].pose[:3, 3], [5.0, -3.0, 1.0])
    np.testing.assert_allclose(lfv._base_link_axis.pose[:3, 3], [5.0, -3.0, 1.0])


def _make_lidar_info_and_frame():
    """A small lidar meta + frame with one valid RANGE pixel."""
    meta = core.LidarInfo.from_default(core.LidarMode._1024x10)
    frame = core.LidarFrame(meta)
    frame.status[:] = 1
    frame.timestamp[:] = np.arange(frame.w)
    frame.field(core.ChanField.RANGE)[:] = 0
    frame.field(core.ChanField.RANGE)[0, 0] = 1000
    return meta, frame


def test_interleaved_camera_before_lidar_frame_routing():
    """A mixed FrameSet where the camera is registered *before* the lidar must
    still route each frame to the correct model.

    OSF collation exposes sensor_info()/FrameSet frames in registration order,
    which may interleave lidar and camera streams. The viz must map frames to
    models by recorded position, not assume lidars-first-then-cameras.
    """
    viz = MockPointViz()
    camera_info = _make_camera_info()
    lidar_meta, lidar_frame = _make_lidar_info_and_frame()

    # camera FIRST, lidar SECOND
    model = LidarFrameVizModel(viz, [camera_info, lidar_meta])

    assert model._camera_frame_indices == [0]
    assert model._sensor_frame_indices == [1]

    camera_frame = _make_camera_frame()
    frames = core.FrameSet([camera_frame, lidar_frame])

    # each stream's frame must be pulled by recorded position; a lidars-first
    # assumption would swap these (and return the wrong frame type).
    lidar_frames = model._lidar_frames(frames)
    camera_frames = model._camera_frames(frames)
    assert len(lidar_frames) == 1 and isinstance(lidar_frames[0], core.LidarFrame)
    assert len(camera_frames) == 1 and isinstance(camera_frames[0], core.CameraFrame)
    assert camera_frames[0].w == 4 and camera_frames[0].h == 3

    # a full update must run and project the camera RANGE cloud
    model.update(frames)
    camera = model._cameras[0]
    assert camera._w == camera_frame.w and camera._h == camera_frame.h
    expected = np.asarray(
        core.XYZLut(_make_camera_info())(camera_frame),
        dtype=np.float32).reshape(-1, 3)
    np.testing.assert_allclose(expected[0], [2.0, 0.0, 0.0], atol=1e-5)


def test_interleaved_camera_objects_routed_with_extrinsics():
    """Camera per-frame objects get the camera sensor->world transform even when
    the camera precedes a lidar in the FrameSet (interleaved ordering)."""
    viz = MockPointViz()
    camera_info = _make_camera_info()
    sensor_to_body = np.eye(4)
    sensor_to_body[:3, 3] = [1.0, 0.0, 2.0]
    camera_info.sensor_to_body = sensor_to_body
    lidar_meta, lidar_frame = _make_lidar_info_and_frame()

    # camera FIRST, lidar SECOND
    model = LidarFrameVizModel(viz, [camera_info, lidar_meta])

    body_to_world = np.eye(4)
    body_to_world[:3, 3] = [10.0, 0.0, 0.0]
    camera_frame = _make_camera_frame()
    camera_frame.body_to_world[0] = body_to_world

    obj = core.Object()
    obj.id = 7
    obj.class_id = 0
    obj.velocity = np.zeros(3)
    obj.dimensions = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    obj.object_to_body.position = np.array([3.0, 0.0, 0.0])
    camera_frame.objects['objects'] = [obj]

    model.update_objects(core.FrameSet([camera_frame, lidar_frame]))

    camera = model._cameras[0]
    sensor_to_world = camera.camera_to_world(camera_frame)
    assert len(model._objects) == 1
    rendered_obj = next(iter(model._objects))
    np.testing.assert_allclose(rendered_obj.body_to_world.to_matrix(), sensor_to_world)
    world_pos = (rendered_obj.body_to_world * rendered_obj.object_to_body).position
    np.testing.assert_allclose(world_pos, [14.0, 0.0, 2.0], atol=1e-6)


def test_cycle_cloud_mode_redraws_paused_mixed_scene():
    """Cycling the cloud mode while paused must re-render the current frame set
    in a mixed lidar+camera scene.

    Regression: the paused-redraw guard compared len(frame_set) against the
    lidar count only, so with a camera also present the counts never matched and
    model.update() (which re-colors the shown frame) was skipped -- the view
    froze on mode changes. A lidar-only file happened to match, so it worked
    there. The guard now counts all streams (lidars + cameras)."""
    from ouster.sdk.viz.core import LidarFrameViz

    viz = MockPointViz()
    lidar_meta, lidar_frame = _make_lidar_info_and_frame()
    camera_info = _make_camera_info()
    lfv = LidarFrameViz([lidar_meta, camera_info], viz, _add_default_controls=False)

    frames = core.FrameSet([lidar_frame, _make_camera_frame()])
    lfv.update(frames, frame_num=0)

    # Spy on the model's re-render after the initial update() above.
    calls = []
    orig_update = lfv._model.update

    def spy(fs, new_frames=True, *args, **kwargs):
        calls.append(new_frames)
        return orig_update(fs, new_frames, *args, **kwargs)
    lfv._model.update = spy

    # Simulate a keypress that cycles the cloud mode while paused.
    lfv.cycle_cloud_mode()

    # The current (paused) frame set must be re-colored: model.update(..., False).
    assert any(nf is False for nf in calls), \
        "cycling cloud mode did not re-render the paused mixed frame set"


def test_frame_period_with_a_camera_reporting_no_fps():
    """Playback timing must survive a camera whose metadata carries no frame rate.

    Camera recordings made before fps was serialized report 0, and cameras may
    be registered before lidars, so the first sensor info is not necessarily one
    with a usable rate.
    """
    from unittest.mock import patch
    from ouster.sdk.viz import SimpleViz

    camera_info = _make_camera_info()
    assert camera_info.fps == 0
    lidar_meta, _ = _make_lidar_info_and_frame()

    with patch('ouster.sdk.viz.core.add_default_controls'):
        # camera alone: nothing reports a rate, so fall back to 30 Hz
        sv = SimpleViz([camera_info], _override_pointviz=MockPointViz())
        assert sv._frame_period() == 1.0 / 30.0

        # camera registered first: take the rate from the lidar behind it
        sv = SimpleViz([camera_info, lidar_meta], _override_pointviz=MockPointViz())
        assert sv._frame_period() == 1.0 / lidar_meta.fps

        # a single info rather than a sequence is still accepted
        sv = SimpleViz(lidar_meta, _override_pointviz=MockPointViz())
        assert sv._frame_period() == 1.0 / lidar_meta.fps
