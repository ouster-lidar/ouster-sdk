import sys
import pytest
from unittest.mock import MagicMock, patch
from ouster.sdk.core import LidarFrame, SensorInfo, LidarMode

from .conftest import MockPointViz


def test_viz_deprecations(capsys):
    from ouster.sdk.viz import LidarScanViz, ls_show, LidarScanVizAccumulatorsConfig
    sensor_info = SensorInfo.from_default(LidarMode._1024x10)
    with pytest.warns(FutureWarning, match="LidarScanViz is deprecated: "
        "Use LidarFrameViz instead. The last supported version for this will be 1.0."):
        LidarScanViz([sensor_info], MockPointViz())
    with pytest.warns(FutureWarning, match="LidarScanVizAccumulatorsConfig is deprecated: "
        "Use LidarFrameVizAccumulatorsConfig instead. The last supported version for this will be 1.0."):
        LidarScanVizAccumulatorsConfig([sensor_info])

    frame = LidarFrame(sensor_info)
    mock_simple_viz_instance = MagicMock()
    mock_simple_viz = MagicMock(return_value=mock_simple_viz_instance)
    with patch('ouster.sdk.viz.core.SimpleViz', mock_simple_viz):
        with pytest.warns(FutureWarning, match="ls_show is deprecated: "
            "Use lf_show instead. The last supported version for this will be 1.0."):
            ls_show(frame)
    mock_simple_viz.assert_called_once()
    mock_simple_viz_instance.run.assert_called_once()


def test_scan_record_deprecated_alias():
    from ouster.sdk.viz.track import ScanRecord, FrameRecord
    with pytest.warns(FutureWarning, match="ScanRecord is deprecated: "
        "Use FrameRecord instead. The last supported version for this will be 1.0."):
        record = ScanRecord(pose=None, frame=None)
    assert isinstance(record, FrameRecord)


def test_scans_accumulator_deprecated_alias():
    from ouster.sdk.viz.frames_accumulator import ScansAccumulator
    with pytest.warns(FutureWarning, match="ScansAccumulator is deprecated: "
        "Use FramesAccumulator instead. The last supported version for this will be 1.0."):
        # The warning fires before the wrapped constructor runs; constructing a
        # FramesAccumulator requires a number of args so tolerate the error.
        with pytest.raises(Exception):
            ScansAccumulator()


def test_scans_accumulator_module_reexports_old_name():
    # The deprecated module shim must re-export the deprecated class name too.
    # A module body only runs once per process, so evict it first to guarantee
    # the import-time warning fires regardless of any earlier import elsewhere
    # in the suite.
    sys.modules.pop("ouster.sdk.viz.scans_accumulator", None)
    with pytest.warns(FutureWarning, match="ouster.sdk.viz.scans_accumulator is deprecated"):
        from ouster.sdk.viz.scans_accumulator import ScansAccumulator  # noqa: F401


def test_lidar_scan_viz_model_deprecated_alias():
    from ouster.sdk.viz.model import LidarScanVizModel
    with pytest.warns(FutureWarning, match="LidarScanVizModel is deprecated: "
        "Use LidarFrameVizModel instead. The last supported version for this will be 1.0."):
        viz = MockPointViz()
        sensor_info = SensorInfo.from_default(LidarMode._1024x10)
        LidarScanVizModel(viz, [sensor_info], _img_aspect_ratio=1.0)


def test_lidar_scan_viz_accumulators_deprecated_alias():
    from ouster.sdk.viz.accumulators import LidarScanVizAccumulators
    with pytest.warns(FutureWarning, match="LidarScanVizAccumulators is deprecated: "
        "Use LidarFrameVizAccumulators instead. The last supported version for this will be 1.0."):
        # The warning fires before the wrapped constructor runs; constructing a
        # LidarScanVizAccumulators requires a number of args so tolerate the error.
        with pytest.raises(Exception):
            LidarScanVizAccumulators()
