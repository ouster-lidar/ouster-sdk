"""
Tests for deprecated type/function name aliases and import-time deprecations.
"""

import importlib
import sys
from unittest.mock import patch, MagicMock

import pytest

from ouster.sdk import core
from ouster.sdk.core import LidarFrame, LidarMode, SensorInfo, FrameSet, \
    FrameSetSource, scan_to_packets, PacketFormat, LidarPacket, FrameBatcher
from ouster.sdk.viz import SimpleViz

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


@pytest.mark.parametrize("deprecated_name,new_name,invoke", [
    ("LidarScan", "LidarFrame", lambda meta, frame: core.LidarScan(meta)),  # type: ignore[attr-defined]
    ("LidarScanSet", "FrameSet", lambda meta, frame: core.LidarScanSet([frame])),  # type: ignore[attr-defined]
    ("ScanBatcher", "FrameBatcher", lambda meta, frame: core.ScanBatcher(meta)),  # type: ignore[attr-defined]
    ("ScanSource", "FrameSetSource", lambda meta, frame: core.ScanSource()),  # type: ignore[attr-defined]
    ("MultiScanSource", "MultiFrameSetSource",
     lambda meta, frame: core.MultiScanSource([])),  # type: ignore[attr-defined]
])
def test_core_deprecated_aliases_warn(deprecated_name, new_name, invoke, meta, frame):
    with pytest.warns(
        FutureWarning,
        match=f"{deprecated_name} is deprecated: Use {new_name} instead",
    ):
        invoke(meta, frame)


def test_clipped_scan_source_deprecated_alias_warns(frame):
    with pytest.warns(
        FutureWarning,
        match="ClippedScanSource is deprecated: Use ClippedFrameSetSource instead",
    ):
        core.ClippedScanSource(
            FrameSet([frame]),
            fields=["RANGE"],
            lower=0,
            upper=1000,
        )


def test_masked_scan_source_deprecated_alias_warns(minimal_source):
    with pytest.warns(
        FutureWarning,
        match="MaskedScanSource is deprecated: Use MaskedFrameSetSource instead",
    ):
        core.MaskedScanSource(
            minimal_source,
            fields=[],
            masks=[None],
        )


def test_reduced_scan_source_deprecated_alias_warns(minimal_source, meta):
    with pytest.warns(
        FutureWarning,
        match="ReducedScanSource is deprecated: Use ReducedFrameSetSource instead",
    ):
        core.ReducedScanSource(minimal_source, [meta.format.pixels_per_column])


def test_pcap_scan_source_deprecated_alias_warns():
    from ouster.sdk import pcap

    with pytest.warns(
        FutureWarning,
        match="PcapScanSource is deprecated: Use PcapFrameSetSource instead",
    ):
        with pytest.raises(Exception):
            pcap.PcapScanSource("/nonexistent/path.pcap")


def test_sensor_scan_source_deprecated_alias_warns():
    from ouster.sdk import sensor

    with pytest.warns(
        FutureWarning,
        match="SensorScanSource is deprecated: Use SensorFrameSetSource instead",
    ):
        with pytest.raises(Exception):
            sensor.SensorScanSource("127.0.0.1")


def test_bag_scan_source_deprecated_alias_warns():
    from ouster.sdk import bag

    with pytest.warns(
        FutureWarning,
        match="BagScanSource is deprecated: Use BagFrameSetSource instead",
    ):
        with pytest.raises(Exception):
            bag.BagScanSource("/nonexistent/path.bag")


def test_osf_scan_source_deprecated_alias_warns():
    from ouster.sdk import osf

    with pytest.warns(
        FutureWarning,
        match="OsfScanSource is deprecated: Use OsfFrameSetSource instead",
    ):
        with pytest.raises(Exception):
            osf.OsfScanSource("/nonexistent/path.osf")


def test_osf_drop_scan_error_deprecated_alias():
    from ouster.sdk import osf

    with pytest.warns(
        FutureWarning,
        match="OsfDropScanError is deprecated: Use OsfDropFrameError instead",
    ):
        osf.OsfDropScanError()


def test_osf_png_lidar_scan_encoder_deprecated_alias():
    from ouster.sdk import osf

    with pytest.warns(
        FutureWarning,
        match="PngLidarScanEncoder is deprecated: Use PngLidarFrameEncoder instead",
    ):
        encoder = osf.PngLidarScanEncoder(0)
    assert isinstance(encoder, osf.PngLidarFrameEncoder)


def test_osf_zpng_lidar_scan_encoder_deprecated_alias():
    from ouster.sdk import osf

    with pytest.warns(
        FutureWarning,
        match="ZPngLidarScanEncoder is deprecated: Use ZPngLidarFrameEncoder instead",
    ):
        encoder = osf.ZPngLidarScanEncoder(0)
    assert isinstance(encoder, osf.ZPngLidarFrameEncoder)


def test_osf_lidar_scan_stream_deprecated_alias():
    from ouster.sdk import osf

    with pytest.warns(
        FutureWarning,
        match="LidarScanStream is deprecated: Use LidarFrameStream instead",
    ):
        # Calling the alias triggers the warning; constructing a stream
        # directly is not a supported operation, so any error after the
        # warning is fine.
        with pytest.raises(Exception):
            osf.LidarScanStream()


def test_osf_encoder_lidar_scan_encoder_kwarg_deprecated():
    from ouster.sdk import osf

    inner = osf.PngLidarFrameEncoder(0)
    with pytest.warns(
        FutureWarning,
        match="Keyword argument 'lidar_scan_encoder' is deprecated, "
              "use 'lidar_frame_encoder' instead. "
              "The last supported version for this will be 1.0.",
    ):
        encoder = osf.Encoder(lidar_scan_encoder=inner)
    assert isinstance(encoder, osf.Encoder)


def test_frame_set_valid_scans_deprecated(frame):
    frame_set = FrameSet([frame])
    with pytest.warns(FutureWarning, match="valid_scans is deprecated, use valid_frames"):
        frame_set.valid_scans()


def test_frame_set_source_scans_num_deprecated(real_pcap_path):
    from contextlib import closing
    from ouster.sdk import open_source

    with closing(open_source(real_pcap_path, index=True)) as source:
        with pytest.warns(FutureWarning, match="scans_num is deprecated, use frames_num"):
            source.scans_num


def test_seekable_scan_num_deprecated(meta, frame):
    from ouster.sdk.viz.core import _Seekable

    seekable = _Seekable(iter([FrameSet([frame])]), maxlen=1)
    next(seekable)
    with pytest.warns(FutureWarning, match="scan_num is deprecated, use frame_num"):
        assert seekable.scan_num == seekable.frame_num


@pytest.mark.parametrize("module_name,message", [
    ("ouster.sdk.core.scan_ops", "scan_ops is deprecated"),
    ("ouster.sdk.core.clipped_scan_source", "clipped_scan_source is deprecated"),
    ("ouster.sdk.core.masked_scan_source", "masked_scan_source is deprecated"),
    ("ouster.sdk.core.reduced_scan_source", "reduced_scan_source is deprecated"),
    ("ouster.sdk.bag.bag_scan_source", "bag_scan_source is deprecated"),
    ("ouster.sdk.viz.scans_accumulator", "scans_accumulator is deprecated"),
    ("ouster.sdk.examples.lidar_scan", "lidar_scan is deprecated"),
])
def test_deprecated_module_import_warns(module_name, message):
    sys.modules.pop(module_name, None)
    with pytest.warns(FutureWarning, match=message):
        importlib.import_module(module_name)


def test_scan_source_metadata_set_deprecated_alias():
    with pytest.warns(
        FutureWarning,
        match="ScanSourceMetadataSet is deprecated: Use FrameSetSourceMetadataSet instead",
    ):
        core.ScanSourceMetadataSet()


def test_packet_format_from_metadata_deprecated(meta):
    with pytest.warns(FutureWarning, match="from_metadata is deprecated, use from_info instead"):
        PacketFormat.from_metadata(meta)


def test_lidar_frame_hw_constructor_deprecated(meta):
    with pytest.warns(
        FutureWarning,
        match="LidarFrame\\(h, w\\) is deprecated, use LidarFrame\\(h, w, field_types, columns_per_packet\\)",
    ):
        LidarFrame(meta.format.pixels_per_column, meta.format.columns_per_frame)


def test_frame_batcher_call_deprecated(meta):
    packet_format = PacketFormat(meta)
    batcher = FrameBatcher(meta)
    frame = LidarFrame(meta)
    packet = LidarPacket(packet_format)
    packet_format.set_col_measurement_id(packet, 0, 0)
    with pytest.warns(
        FutureWarning,
        match="FrameBatcher.__call__\\(\\) is deprecated, use FrameBatcher.batch\\(\\) instead",
    ):
        batcher(packet, frame)


def test_deprecated_scan_to_packets(frame, meta):
    pf = PacketFormat(meta)
    with pytest.warns(FutureWarning, match="scan_to_packets is deprecated: Use frame_to_packets instead."):
        scan_to_packets(lidar_frame=frame, packet_format=pf, init_id=0, prod_sn=0)


def test_deprecated_util_scan_to_packets(frame, meta):
    # ouster.sdk.util.scan_to_packets wraps the pure-Python
    # ouster.sdk.util.parsing.frame_to_packets, whose signature is (lf, info) --
    # distinct from the core binding of the same name.
    from ouster.sdk.util import scan_to_packets as util_scan_to_packets
    with pytest.warns(FutureWarning, match="scan_to_packets is deprecated: Use frame_to_packets instead."):
        util_scan_to_packets(frame, meta)


def test_deprecated_scans_per_sec(frame, meta):
    # add_default_controls is a native binding that rejects the MockPointViz
    # injected via _override_pointviz, so patch it out during construction.
    with patch('ouster.sdk.viz.core.add_default_controls'):
        viz = SimpleViz([meta], _override_pointviz=MockPointViz())
    with pytest.warns(FutureWarning, match="scans_per_sec is deprecated, use frames_per_sec"):
        viz.scans_per_sec


def test_perception_normals_warns_on_call():
    """normals() is a wrapper; the FutureWarning fires when the function is called."""
    import ouster.sdk.perception as perception
    with patch.object(perception, "_normals", MagicMock()):
        with pytest.warns(FutureWarning, match="ouster.sdk.perception.normals"):
            perception.normals()
