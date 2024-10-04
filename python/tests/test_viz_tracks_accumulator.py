from .conftest import MockPointViz
from ouster.sdk.client import SensorInfo, LidarMode, LidarScan
from ouster.sdk.viz.model import LidarScanVizModel
from ouster.sdk.viz.accumulators_config import LidarScanVizAccumulatorsConfig
from ouster.sdk.viz.track import Track, TRACK_INIT_POINTS_NUM
from ouster.sdk.viz.tracks_accumulator import TracksAccumulator


def test_toggle_visibility():
    """It should add/remove the Clouds used to render
    the track and key frames depending on visibility."""
    infos = [
        SensorInfo.from_default(LidarMode.MODE_2048x10),
        SensorInfo.from_default(LidarMode.MODE_2048x10)
    ]
    model = LidarScanVizModel(infos, _img_aspect_ratio=0)
    config = LidarScanVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=2)
    track = Track(config)
    viz = MockPointViz()
    accum = TracksAccumulator(model, viz, track)
    assert accum.track_visible  # visible by default
    assert (accum._cloud_track in viz.items) == accum.track_visible
    assert (accum._cloud_kf_track in viz.items) == accum.track_visible
    accum.toggle_visibility()   # hide
    assert not accum.track_visible
    assert (accum._cloud_track in viz.items) == accum.track_visible
    assert (accum._cloud_kf_track in viz.items) == accum.track_visible
    accum.toggle_visibility()   # show
    assert accum.track_visible
    assert (accum._cloud_track in viz.items) == accum.track_visible
    assert (accum._cloud_kf_track in viz.items) == accum.track_visible


def test_draw_track():
    """It should update the cloud based on the contents of the track."""
    infos = [
        SensorInfo.from_default(LidarMode.MODE_2048x10),
        SensorInfo.from_default(LidarMode.MODE_2048x10)
    ]
    model = LidarScanVizModel(infos, _img_aspect_ratio=0)
    config = LidarScanVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=2)
    track = Track(config)
    assert len(track._xyz) == TRACK_INIT_POINTS_NUM
    viz = MockPointViz()
    accum = TracksAccumulator(model, viz, track)
    assert accum._cloud_track.size == TRACK_INIT_POINTS_NUM

    for scan_num in range(TRACK_INIT_POINTS_NUM):
        track.update(LidarScan(1, 1), scan_num)
    track.update(LidarScan(1, 1), scan_num + 1)

    accum._draw_track()

    # The cloud was replaced by a bigger one
    # TODO[tws] add a more assertive assertion
    assert accum._cloud_track.size > TRACK_INIT_POINTS_NUM
