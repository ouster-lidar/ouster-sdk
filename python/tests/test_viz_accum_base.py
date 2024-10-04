from .conftest import MockPointViz
from ouster.sdk.viz.accum_base import AccumulatorBase
from ouster.sdk.client import SensorInfo, LidarMode, LidarScan, ChanField
from ouster.sdk.viz.model import LidarScanVizModel
from ouster.sdk.viz.accumulators_config import LidarScanVizAccumulatorsConfig
from ouster.sdk.viz.track import MultiTrack


def test_use_default_view_modes():
    """It should add/remove the Clouds used to render
    the track and key frames depending on visibility."""
    infos = [
        SensorInfo.from_default(LidarMode.MODE_2048x10),
        SensorInfo.from_default(LidarMode.MODE_2048x10)
    ]
    for info in infos:
        info.image_rev = 'ousteros-image-prod-bootes-v3.0.1'  # needed for "is_norm_reflectivity_mode"
    model = LidarScanVizModel(infos, _img_aspect_ratio=0)
    config = LidarScanVizAccumulatorsConfig(accum_max_num=2, accum_min_dist_num=2)
    track = MultiTrack(model, config)
    viz = MockPointViz()
    accum = AccumulatorBase(model, viz, track)
    assert accum.metadata == infos
    assert accum.active_cloud_mode == ''
    assert accum.get_palette(accum.active_cloud_mode).name == 'Cal. Ref'

    # no cloud mode name is set initially and none are available until the first scan is read
    accum._use_default_view_modes()
    assert accum.active_cloud_mode == accum._cloud_mode_name == ''

    model._amend_view_modes_all([LidarScan(1, 1) for _ in infos])

    accum._use_default_view_modes()
    assert accum.active_cloud_mode == ChanField.REFLECTIVITY
    accum.cycle_cloud_palette()

    assert accum.get_palette(
        model._sensors[0]._cloud_modes.get(accum.active_cloud_mode)
    ).name == 'Cal. Ref. Ouster Colors'

    accum.cycle_cloud_mode()
    assert accum.active_cloud_mode == ChanField.SIGNAL

    # Only REFLECTIVITY uses Cal Ref palettes; other channels use regular palettes
    assert accum.get_palette(
        model._sensors[0]._cloud_modes.get(accum.active_cloud_mode)
    ).name == 'Ouster Colors'

    # has no effect after the active cloud mode is set
    accum._use_default_view_modes()
    assert accum.active_cloud_mode == ChanField.SIGNAL

    accum.cycle_cloud_mode()
    assert accum.active_cloud_mode == ChanField.NEAR_IR
