from .conftest import MockPointViz
from ouster.sdk.client import SensorInfo, LidarMode, LidarScan, ChanField
from ouster.sdk.viz.model import LidarScanVizModel
from ouster.sdk.viz.accumulators_config import LidarScanVizAccumulatorsConfig
from ouster.sdk.viz.track import MultiTrack
from ouster.sdk.viz.scans_accumulator import ScansAccumulator
from ouster.sdk.viz.view_mode import ReflMode


def test_toggle_visibility():
    """It should add/remove the Clouds used to render
    the track and key frames depending on visibility."""
    infos = [
        SensorInfo.from_default(LidarMode.MODE_2048x10),
        SensorInfo.from_default(LidarMode.MODE_2048x10)
    ]
    model = LidarScanVizModel(infos, _img_aspect_ratio=0)
    config = LidarScanVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=2)
    track = MultiTrack(model, config)
    viz = MockPointViz()
    accum = ScansAccumulator(model, viz, track)

    # must have view modes updated before anything can be added to the viz
    accum._cloud_mode_name = ChanField.REFLECTIVITY
    model._sensors[0]._cloud_modes[ChanField.REFLECTIVITY] = ReflMode()
    model._sensors[1]._cloud_modes[ChanField.REFLECTIVITY] = ReflMode()

    assert len(viz.items) == 0  # no key frames yet
    assert accum.accum_visible  # visible by default

    accum.toggle_visibility()
    assert not accum.accum_visible

    # add some scans
    scans = [LidarScan(info.h, info.w) for info in infos]
    track.update(scans)
    accum.update(scans)

    accum.toggle_visibility()
    assert accum.accum_visible

    # TODO[tws] improve this test and its assertions
    # there should be a cloud for each key frame
    assert len(viz.items) == len(infos)

    # Disabling/enabling sensors removes/adds their clouds to the viz
    model._sensors[0]._enabled = False
    accum.update(scans)
    assert len(viz.items) == len(infos) - 1

    model._sensors[0]._enabled = True
    accum.update(scans)
    assert len(viz.items) == len(infos)
