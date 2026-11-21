import numpy as np

from .conftest import MockPointViz
from ouster.sdk import core
from ouster.sdk.core import LidarInfo, LidarMode, LidarFrame, ChanField
from ouster.sdk._bindings.viz import _cloud_lut_cache_misses
from ouster.sdk.viz.model import LidarFrameVizModel
from ouster.sdk.viz.accumulators_config import LidarFrameVizAccumulatorsConfig
from ouster.sdk.viz.track import MultiTrack
from ouster.sdk.viz.frames_accumulator import FramesAccumulator
from ouster.sdk.viz.view_mode import ReflMode


def test_toggle_visibility():
    """It should add/remove the Clouds used to render
    the track and key frames depending on visibility."""
    infos = [
        LidarInfo.from_default(LidarMode._2048x10),
        LidarInfo.from_default(LidarMode._2048x10)
    ]
    viz = MockPointViz()
    model = LidarFrameVizModel(viz, infos, _img_aspect_ratio=0)
    config = LidarFrameVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=2)
    track = MultiTrack(model, config)
    viz = MockPointViz()
    accum = FramesAccumulator(model, viz, track)

    # must have view modes updated before anything can be added to the viz
    accum._cloud_mode_name = ChanField.REFLECTIVITY
    model._sensors[0]._cloud_modes[ChanField.REFLECTIVITY] = ReflMode()
    model._sensors[1]._cloud_modes[ChanField.REFLECTIVITY] = ReflMode()

    assert len(viz.items) == 0  # no key frames yet
    assert accum.accum_visible  # visible by default

    accum.toggle_visibility()
    assert not accum.accum_visible

    # add some frames
    frames = [LidarFrame(info) for info in infos]
    track.update(frames)
    accum.update(frames)

    accum.toggle_visibility()
    assert accum.accum_visible

    # TODO[tws] improve this test and its assertions
    # there should be a cloud for each key frame
    assert len(viz.items) == len(infos)

    # Disabling/enabling sensors removes/adds their clouds to the viz
    model._sensors[0]._enabled = False
    accum.update(frames)
    assert len(viz.items) == len(infos) - 1

    model._sensors[0]._enabled = True
    accum.update(frames)
    assert len(viz.items) == len(infos)


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


def _make_camera_info(w=4, h=3):
    cam_info = core.CameraInfo()
    cam_info.model = core.CameraModel(100.0, 100.0, (w - 1) / 2.0, (h - 1) / 2.0)
    cam_info.sensor_to_body = np.eye(4)
    cam_info.format.columns_per_frame = w
    cam_info.format.pixels_per_column = h
    # The shared XYZLut derives per-pixel direction/offset from the pinhole
    # intrinsics, so no beam angles are stored on the CameraInfo.
    return cam_info


def _make_camera_frame(cam_info, w=4, h=3, pose=None):
    frame = core.CameraFrame(cam_info)
    frame.w = w
    frame.h = h
    depth = np.full((h, w), 2.0, dtype=np.float64)
    _, _, mag = _camera_beam_angles(w, h, 100.0, 100.0, (w - 1) / 2.0, (h - 1) / 2.0)
    range_mm = np.ascontiguousarray(
        np.where(depth > 0.0, depth * mag * 1000.0, 0.0).astype(np.uint32))
    frame.add_field("RANGE", range_mm)
    frame.body_to_world[0] = np.eye(4) if pose is None else pose
    return frame


def test_camera_accumulates_key_frame_clouds():
    """Camera RANGE key frames should be projected into world-frame accum clouds."""
    cam_info = _make_camera_info()
    viz = MockPointViz()
    model = LidarFrameVizModel(viz, [cam_info])
    config = LidarFrameVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=1)
    track = MultiTrack(model, config)
    accum = FramesAccumulator(model, viz, track)

    frame = _make_camera_frame(cam_info)
    frames = core.FrameSet([frame])
    model.update(frames)  # builds camera geometry + RANGE cloud mode
    accum._cloud_mode_name = "RANGE"

    track.update(frames)
    accum.update(frames)

    camera_clouds = accum._camera_clouds[0]
    added = [c for c in camera_clouds._clouds_accum if c is not None]
    assert len(added) == 1
    assert added[0].size == 12
    assert added[0] in viz.items


def test_cloud_lut_cache_is_stable_while_accumulating():
    """Accumulating many frames from a lidar and a camera builds no new cloud buffers once
    the first frame is shown, and each model keeps projecting with its sensor info's cached
    LUT rather than building its own. The cache is process-wide, so this counts misses,
    which clouds released by other tests or threads can't change, rather than entries."""
    lidar_info = LidarInfo.from_default(LidarMode._512x10)
    cam_info = _make_camera_info()
    viz = MockPointViz()
    model = LidarFrameVizModel(viz, [lidar_info, cam_info])
    config = LidarFrameVizAccumulatorsConfig(accum_max_num=3, accum_min_dist_num=1)
    track = MultiTrack(model, config)
    accum = FramesAccumulator(model, viz, track)
    accum._cloud_mode_name = "RANGE"

    def step(i):
        pose = np.eye(4)
        pose[0, 3] = float(i)
        lidar_frame = LidarFrame(lidar_info)
        lidar_frame.field(ChanField.RANGE)[:] = 5000
        lidar_frame.body_to_world[:] = pose
        frames = core.FrameSet([lidar_frame, _make_camera_frame(cam_info, pose=pose)])
        model.update(frames)
        track.update(frames)
        accum.update(frames)

    step(0)
    misses = _cloud_lut_cache_misses()
    for i in range(1, 10):
        step(i)
        assert model._sensors[0]._xyzlut is lidar_info.xyzlut_double, f"lidar LUT replaced at frame {i}"
        assert model._cameras[0]._xyzlut is cam_info.xyzlut_double, f"camera LUT replaced at frame {i}"
        assert _cloud_lut_cache_misses() == misses, f"cloud buffers rebuilt at frame {i}"

    # key frames rotated through the accumulator the whole time
    assert sum(c is not None for c in accum._sensor_clouds[0]._clouds_accum) == 3
    assert sum(c is not None for c in accum._camera_clouds[0]._clouds_accum) == 3


def test_camera_accum_toggle_visibility():
    """Toggling accum visibility should add/remove camera key-frame clouds."""
    cam_info = _make_camera_info()
    viz = MockPointViz()
    model = LidarFrameVizModel(viz, [cam_info])
    config = LidarFrameVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=1)
    track = MultiTrack(model, config)
    accum = FramesAccumulator(model, viz, track)

    frame = _make_camera_frame(cam_info)
    frames = core.FrameSet([frame])
    model.update(frames)
    accum._cloud_mode_name = "RANGE"
    track.update(frames)
    accum.update(frames)

    camera_cloud = accum._camera_clouds[0]._clouds_accum[0]
    assert camera_cloud in viz.items

    accum.toggle_visibility()  # hide
    assert not accum.accum_visible
    assert camera_cloud not in viz.items

    accum.toggle_visibility()  # show
    assert accum.accum_visible
    assert camera_cloud in viz.items
