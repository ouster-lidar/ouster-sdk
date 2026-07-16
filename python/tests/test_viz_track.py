from .conftest import MockPointViz
from ouster.sdk.core import LidarFrame, first_valid_column_pose, SensorInfo, LidarMode
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
        SensorInfo.from_default(LidarMode._2048x10),
        SensorInfo.from_default(LidarMode._2048x10)
    ]
    viz = MockPointViz()
    model = LidarFrameVizModel(viz, infos, _img_aspect_ratio=0)
    config = LidarFrameVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=2)
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
        pose = first_valid_column_pose(frame)
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
