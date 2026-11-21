from .conftest import MockPointViz
import numpy as np
from ouster.sdk.core import (CameraFrame, CameraInfo, FrameSet, LidarFrame,
                             CameraModel, LidarInfo, LidarMode)
from ouster.sdk.viz.model import LidarFrameVizModel
from ouster.sdk.viz.track import Track, MultiTrack, TRACK_INIT_POINTS_NUM, TRACK_MAP_GROWTH_RATE
from ouster.sdk.viz.accumulators_config import LidarFrameVizAccumulatorsConfig


def test_track_init_defaults():
    """It should set some defaults."""
    config = LidarFrameVizAccumulatorsConfig()
    track = Track(config)
    assert track._xyz.shape[0] == 100
    assert track._key.shape[0] == 100
    assert track._kf_xyz.shape[0] == 1
    assert track._kf_key.shape[0] == 1
    assert len(track._frame_records) == 0


def test_track_update():
    """Calling update with a new frame adds the frame to the frame records
    list."""
    config = LidarFrameVizAccumulatorsConfig()
    track = Track(config)
    frame = LidarFrame(1, 1, [], 16)
    assert len(track._frame_records) == 0
    assert track._frame_num == -1
    track.update(frame, 0)
    assert len(track._frame_records) == 1
    assert track._frame_num == 0
    track.update(frame, 1)
    assert len(track._frame_records) == 2
    assert track._frame_num == 1
    track.update(frame, 2)
    assert len(track._frame_records) == 3
    assert track._frame_num == 2


def test_track_update_2():
    """Calling update with a previously-seen frame does not add it to the frame records
    list."""
    config = LidarFrameVizAccumulatorsConfig()
    track = Track(config)
    frame = LidarFrame(1, 1, [], 16)
    assert len(track._frame_records) == 0
    assert track._frame_num == -1
    track.update(frame, 0)
    assert len(track._frame_records) == 1
    assert track._frame_num == 0
    track.update(frame, 1)
    assert len(track._frame_records) == 2
    assert track._frame_num == 1

    track.update(frame, 0)
    assert len(track._frame_records) == 2
    assert track._frame_num == 0


def test_track_update_3():
    """It should expand the list of points used for the track."""
    # TODO[tws]: we should either use fixed-sized data structures, or we should encapsulate the growth and overflow
    # behavior of the data structures.  Mixing the growth logic with the accumulator business logic is an example of
    # unnecessary coupling.
    config = LidarFrameVizAccumulatorsConfig()
    track = Track(config)

    # preconditions
    assert track._xyz.shape[0] == TRACK_INIT_POINTS_NUM
    assert track._key.shape[0] == TRACK_INIT_POINTS_NUM
    assert track._track_idx == 0
    assert len(track._frame_records) == 0

    # add enough frames to fill up the track
    frame = LidarFrame(1, 1, [], 16)
    for frame_num in range(TRACK_INIT_POINTS_NUM):
        track.update(frame, frame_num)

    assert track._xyz.shape[0] == TRACK_INIT_POINTS_NUM
    assert track._track_idx == TRACK_INIT_POINTS_NUM

    # add one more
    track.update(frame, frame_num)

    new_size = TRACK_INIT_POINTS_NUM * TRACK_MAP_GROWTH_RATE + 1
    assert track._xyz.shape[0] == new_size
    assert track._key.shape[0] == new_size
    assert track._track_idx == TRACK_INIT_POINTS_NUM + 1


def test_track_update_4():
    """It does not update records for frames that have already been logged (according to frame_num!)."""
    config = LidarFrameVizAccumulatorsConfig(accum_max_num=100)
    track = Track(config)
    frame = LidarFrame(1, 1, [], 16)
    frame2 = LidarFrame(1, 1, [], 16)
    track.update(frame, 0)
    assert track._frame_records[0].frame == frame
    track.update(frame2, 0)
    assert track._frame_records[0].frame == frame


def test_track_update_5():
    """It only creates FrameRecords for key frames(?).
    This reduces the amount of memory retained for FramesAccumulator cloud color channels (aka cloud_mode_keys)."""
    # TODO[tws] describe this behavior a little better
    config = LidarFrameVizAccumulatorsConfig(accum_max_num=0)
    track = Track(config)
    frame = LidarFrame(1, 1, [], 16)
    frame2 = LidarFrame(1, 1, [], 16)
    track.update(frame, 0)
    assert track._frame_records[0] is None
    track.update(frame2, 0)
    assert track._frame_records[0] is None


def test_multitrack_update():
    """It updates a track for each sensor."""
    infos = [
        LidarInfo.from_default(LidarMode._2048x10),
        LidarInfo.from_default(LidarMode._2048x10)
    ]
    viz = MockPointViz()
    model = LidarFrameVizModel(viz, infos, _img_aspect_ratio=0)
    config = LidarFrameVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=1)
    multitrack = MultiTrack(model, config)
    assert len(multitrack._tracks) == len(infos)

    # create a frame for each sensor
    frames = [LidarFrame(1, 1, [], 16) for _ in infos]

    # update the tracks with the frames
    multitrack.update(frames, 0)
    for track, frame in zip(multitrack._tracks, frames):
        assert track._frame_records[0].frame == frame

    # omitting frame_num increases frame_num by 1
    multitrack.update(frames)
    for track, frame in zip(multitrack._tracks, frames):
        assert track._frame_records[1].frame == frame


def test_key_frames():
    """It should add a key frame every "accum_min_dist_num" frames."""
    config = LidarFrameVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=2)
    track = Track(config)

    # add enough frames to fill up the track
    frames_to_add = 4
    frame = LidarFrame(1, 1, [], 16)
    for frame_num in range(frames_to_add):
        track.update(frame, frame_num)

    assert track._frame_num == frames_to_add - 1
    assert len(track._frame_records) == frames_to_add
    assert track._key_frames == [0, 2, None]

    track.update(frame, track._frame_num + 1)
    assert len(track._frame_records) == frames_to_add + 1
    assert track._key_frames == [None, 2, 4]

    track.update(frame, track._frame_num + 1)
    track.update(frame, track._frame_num + 1)
    assert len(track._frame_records) == frames_to_add + 3
    assert track._key_frames == [6, None, 4]


def test_key_frames_2():
    """It should add a key frame every "accum_min_dist_meters"."""
    config = LidarFrameVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=0, accum_min_dist_meters=2)
    track = Track(config)

    def frame_with_xyz(x, y, z):
        frame = LidarFrame(1, 1, [], 16)
        frame.status[:] = 1
        pose = frame.body_to_world[0]
        pose[0, 3] = x
        pose[1, 3] = y
        pose[2, 3] = z
        return frame

    track.update(frame_with_xyz(0, 0, 0), 0)
    assert track._key_frames == [0, None, None]

    # This point isn't far enough to be a key frame
    track.update(frame_with_xyz(1, 0, 0), 1)
    assert track._key_frames == [0, None, None]

    # This point is far enough to be a key frame
    track.update(frame_with_xyz(config._accum_min_dist_meters + 1, 0, 0), 2)
    assert track._key_frames == [0, 2, None]


def _camera_frame_at(x, y, z):
    frame = CameraFrame()
    pose = np.eye(4)
    pose[0, 3] = x
    pose[1, 3] = y
    pose[2, 3] = z
    frame.body_to_world[0] = pose
    return frame


def test_track_update_camera_frame():
    """Track.update should use CameraFrame.body_to_world[0] as the track pose."""
    config = LidarFrameVizAccumulatorsConfig(accum_max_num=2)
    track = Track(config)
    track.update(_camera_frame_at(1.0, 2.0, 3.0), 0)
    np.testing.assert_allclose(track._xyz[0], [1.0, 2.0, 3.0], atol=1e-5)
    np.testing.assert_allclose(track._frame_records[0].pose[:3, 3], [1.0, 2.0, 3.0])


def test_multitrack_camera_only_update():
    """Camera-only models should keep a camera track and update it from camera_frames."""
    viz = MockPointViz()
    model = LidarFrameVizModel(viz, [CameraInfo()])
    config = LidarFrameVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=1)
    multitrack = MultiTrack(model, config)
    assert len(multitrack._tracks) == 0
    assert len(multitrack._camera_tracks) == 1
    assert multitrack.primary_track is multitrack._camera_tracks[0]

    frames = FrameSet([_camera_frame_at(3.0, 0.0, 0.0)])
    multitrack.update(frames, 0)
    np.testing.assert_allclose(
        multitrack._camera_tracks[0]._xyz[0], [3.0, 0.0, 0.0], atol=1e-5
    )


def test_multitrack_mixed_updates_camera_track():
    """A lidar+camera model should update both lidar and camera tracks."""
    infos = [LidarInfo.from_default(LidarMode._2048x10)]
    viz = MockPointViz()
    model = LidarFrameVizModel(viz, [*infos, CameraInfo()])
    config = LidarFrameVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=1)
    multitrack = MultiTrack(model, config)
    assert len(multitrack._tracks) == 1
    assert len(multitrack._camera_tracks) == 1
    assert len(multitrack.display_tracks()) == 2

    lidar = LidarFrame(1, 1, [], 16)
    frames = FrameSet([lidar, _camera_frame_at(5.0, 0.0, 0.0)])
    multitrack.update(frames, 0)
    assert multitrack._tracks[0]._frame_records[0].frame is lidar
    np.testing.assert_allclose(
        multitrack._camera_tracks[0]._xyz[0], [5.0, 0.0, 0.0], atol=1e-5
    )


def test_lidar_frame_viz_camera_only_creates_track():
    """LidarFrameViz should accumulate a pose track when the source is camera-only."""
    from ouster.sdk.viz.core import LidarFrameViz

    w, h = 4, 3
    fx = fy = 100.0
    camera_info = CameraInfo()
    camera_info.model = CameraModel(fx, fy, 0.0, 0.0)
    camera_info.format.columns_per_frame = w
    camera_info.format.pixels_per_column = h
    # The shared XYZLut derives per-pixel direction/offset from the pinhole
    # intrinsics, so no beam angles are stored on the CameraInfo. The range
    # scaling map |o| still converts planar depth into radial RANGE below.
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    ox = np.ones_like(u, dtype=np.float64)
    oy = -(u / fx)
    oz = -(v / fy)
    mag = np.sqrt(ox ** 2 + oy ** 2 + oz ** 2)

    viz = MockPointViz()
    config = LidarFrameVizAccumulatorsConfig()
    lfv = LidarFrameViz(
        [camera_info], viz, config, _add_default_controls=False
    )
    assert lfv._frames_accum is not None

    cam = _camera_frame_at(0.0, 4.0, 0.0)
    cam.w = w
    cam.h = h
    range_mm = np.ascontiguousarray((2.0 * mag * 1000.0).astype(np.uint32))
    cam.add_field("RANGE", range_mm)
    frames = FrameSet([cam])
    lfv.update(frames, frame_num=0)
    xyz = lfv._frames_accum._track.primary_track._xyz[0]
    np.testing.assert_allclose(xyz, [0.0, 4.0, 0.0], atol=1e-5)
