"""
Tests for deprecated keyword-argument aliases in the scan -> frame migration.
"""

from unittest.mock import patch

import numpy as np
import pytest

from ouster.sdk import core
from ouster.sdk.core import LidarFrame, LidarMode, SensorInfo, FrameSet, frame_to_packets
from ouster.sdk.core import PacketFormat
from ouster.sdk.core import first_valid_column_pose, last_valid_column_pose
from ouster.sdk.core.frame_ops import clip
from ouster.sdk.core.clipped_frame_set_source import ClippedFrameSetSource
from ouster.sdk.core.masked_frame_set_source import MaskedFrameSetSource
from ouster.sdk.core import frame_ops
from ouster.sdk.util.parsing import cut_raw32_words
from ouster.sdk.viz import LidarFrameViz, SimpleViz
from ouster.sdk.viz.track import FrameRecord
from ouster.sdk import algorithm, osf
from ouster.sdk.core import FieldType, ChanField
from ouster.sdk.core import FrameSetSource

from .conftest import MockPointViz


@pytest.fixture
def meta() -> SensorInfo:
    return SensorInfo.from_default(LidarMode._1024x10)


@pytest.fixture
def frame(meta: SensorInfo) -> LidarFrame:
    return LidarFrame(meta)


class _MinimalFrameSetSource(FrameSetSource):
    def __init__(self, info: SensorInfo) -> None:
        FrameSetSource.__init__(self)
        self._info = [info]

    @property
    def sensor_info(self):
        return self._info

    def __iter__(self):
        return iter([])


@pytest.fixture
def minimal_source(meta: SensorInfo) -> FrameSetSource:
    return _MinimalFrameSetSource(meta)


def test_clip_deprecated_scan_kwarg(frame, meta):
    with pytest.warns(FutureWarning, match="Keyword argument 'scan' is deprecated"):
        clip(scan=frame, fields=["RANGE"], lower=0, upper=1000)
        assert frame.field("RANGE").max() <= 1000


def test_clip_deprecated_scan_kwarg_warns(frame):
    with pytest.warns(FutureWarning, match="Keyword argument 'scan' is deprecated"):
        clip(scan=frame, fields=["RANGE"], lower=0, upper=1000)


def test_frame_set_deprecated_scans_kwarg(frame):
    with pytest.warns(FutureWarning, match="Keyword argument 'scans' is deprecated"):
        FrameSet(scans=[frame])


def test_frame_to_packets_deprecated_lidar_scan_kwarg(frame, meta):
    pf = PacketFormat(meta)
    with pytest.warns(FutureWarning, match="Keyword argument 'lidar_scan' is deprecated"):
        frame_to_packets(lidar_scan=frame, packet_format=pf, init_id=0, prod_sn=0)


def test_first_valid_column_pose_deprecated_scan_kwarg(frame):
    with pytest.warns(FutureWarning, match="Keyword argument 'scan' is deprecated"):
        first_valid_column_pose(scan=frame)


def test_clipped_source_deprecated_scan_source_kwarg(frame, meta):
    with pytest.warns(FutureWarning, match="Keyword argument 'scan_source' is deprecated"):
        source = ClippedFrameSetSource(
            scan_source=FrameSet([frame]),
            fields=["RANGE"],
            lower=0,
            upper=1000,
        )
        assert source is not None


def test_clipped_source_deprecated_scan_source_kwarg_warns(frame):
    with pytest.warns(FutureWarning, match="Keyword argument 'scan_source' is deprecated"):
        ClippedFrameSetSource(
            scan_source=FrameSet([frame]),
            fields=["RANGE"],
            lower=0,
            upper=1000,
        )


def test_cut_raw32_words_deprecated_ls_kwarg(frame):
    with pytest.warns(FutureWarning, match="Keyword argument 'ls' is deprecated"):
        cut_raw32_words(ls=frame)


def test_filter_uv_deprecated_scan_kwarg(frame):
    with pytest.warns(FutureWarning, match="Keyword argument 'scan' is deprecated"):
        frame_ops.filter_uv(scan=frame, coord_2d="u", lower=0, upper=1)


def test_lidar_frame_viz_update_deprecated_scans_kwarg(meta, frame):
    viz = LidarFrameViz([meta], MockPointViz())
    with pytest.warns(FutureWarning, match="Keyword argument 'scans' is deprecated"):
        viz.update(scans=FrameSet([frame]))


def test_lidar_frame_viz_update_deprecated_scan_num_kwarg(meta, frame):
    viz = LidarFrameViz([meta], MockPointViz())
    with pytest.warns(FutureWarning, match="Keyword argument 'scan_num' is deprecated"):
        viz.update(frames=FrameSet([frame]), scan_num=3)


def test_simple_viz_deprecated_override_kwarg(meta):
    # add_default_controls is a native binding that rejects the MockPointViz
    # injected via _override_pointviz, so patch it out during construction.
    with patch('ouster.sdk.viz.core.add_default_controls'):
        with pytest.warns(
            FutureWarning,
            match="Keyword argument '_override_lidarscanviz' is deprecated",
        ):
            SimpleViz([meta], _override_lidarscanviz=None, _override_pointviz=MockPointViz())


def test_deprecated_kwarg_conflict_raises(frame):
    with pytest.raises(TypeError, match="Got both deprecated keyword argument 'scan'"):
        clip(scan=frame, frame=frame, fields=["RANGE"], lower=0, upper=1000)


def test_last_valid_column_pose_deprecated_scan_kwarg(frame):
    with pytest.warns(FutureWarning, match="Keyword argument 'scan' is deprecated"):
        last_valid_column_pose(scan=frame)


def test_filter_field_deprecated_scan_kwarg(frame):
    with pytest.warns(FutureWarning, match="Keyword argument 'scan' is deprecated"):
        frame_ops.filter_field(scan=frame, field="RANGE", lower=0, upper=100000)


def test_mask_deprecated_scan_kwarg(frame):
    mask = np.zeros((frame.h, frame.w), dtype=np.uint8)
    with pytest.warns(FutureWarning, match="Keyword argument 'scan' is deprecated"):
        frame_ops.mask(scan=frame, fields=["RANGE"], mask=mask)


def test_select_by_index_deprecated_scan_kwarg(frame):
    with pytest.warns(FutureWarning, match="Keyword argument 'scan' is deprecated"):
        frame_ops.select_by_index(scan=frame, indices=[0], update_metadata=False)


def test_masked_source_deprecated_scan_source_kwarg(minimal_source):
    with pytest.warns(FutureWarning, match="Keyword argument 'scan_source' is deprecated"):
        MaskedFrameSetSource(
            scan_source=minimal_source,
            fields=[],
            masks=[None],
        )


def test_lidar_frame_viz_update_deprecated_last_n_scans_kwarg(meta, frame):
    viz = LidarFrameViz([meta], MockPointViz())
    frames = FrameSet([frame])
    with pytest.warns(FutureWarning, match="Keyword argument 'last_n_scans' is deprecated"):
        viz.update(frames=frames, last_n_scans=[frames])


def test_frame_record_deprecated_scan_kwarg(frame):
    with pytest.warns(FutureWarning, match="Keyword argument 'scan' is deprecated"):
        FrameRecord(pose=np.eye(4), scan=frame)


def test_align_clouds_deprecated_source_scan_kwarg(frame):
    with pytest.warns(FutureWarning, match="Keyword argument 'source_scan' is deprecated"):
        algorithm.align_clouds(source_scan=frame, target_frame=frame)


def test_filter_xyz_deprecated_scan_kwarg(frame, meta):
    lut = core.XYZLut(meta)
    with pytest.warns(FutureWarning, match="Keyword argument 'scan' is deprecated"):
        frame_ops.filter_xyz(
            scan=frame,
            xyzlut=lut,
            axis_idx=0,
            lower=-1000,
            upper=1000,
        )


def test_reduce_by_factor_deprecated_scan_kwarg(frame):
    with pytest.warns(FutureWarning, match="Keyword argument 'scan' is deprecated"):
        frame_ops.reduce_by_factor(scan=frame, factor=2)


def test_reduced_source_deprecated_scan_source_kwarg(minimal_source, meta):
    with pytest.warns(FutureWarning, match="Keyword argument 'scan_source' is deprecated"):
        from ouster.sdk.core.reduced_frame_set_source import ReducedFrameSetSource
        ReducedFrameSetSource(
            scan_source=minimal_source,
            beams=[meta.format.pixels_per_column],
        )


def test_xyz_lut_deprecated_scan_kwarg(frame, meta):
    lut = core.XYZLut(meta)
    with pytest.warns(FutureWarning, match="Keyword argument 'scan' is deprecated"):
        lut(scan=frame)


def test_slice_and_cast_deprecated_lidar_scan_kwarg(frame):
    with pytest.warns(FutureWarning, match="Keyword argument 'lidar_scan' is deprecated"):
        osf.slice_and_cast(
            lidar_scan=frame,
            field_types=[FieldType(ChanField.RANGE, np.uint8)],
        )
