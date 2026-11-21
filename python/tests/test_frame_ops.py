"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

This module verifies that a sliced FrameSetSource matches the original FrameSetSource
but limits the interaction with the source to the scope.
"""

from typing import cast, Union
import os
import pytest
import sys
import tempfile
import copy
from ouster.sdk import open_source, osf, core
from ouster.sdk.core import ChanField, LidarFrame
from ouster.sdk.core import (FrameSetSource, ReducedFrameSetSource,
                             ClippedFrameSetSource, MaskedFrameSetSource,
                             SelectedFrameSetSource)
import ouster.sdk.core.frame_ops as frame_ops
import numpy as np

from tests.conftest import OSFS_DATA_DIR
from tests.conftest import PCAPS_DATA_DIR

paths = [os.path.join(OSFS_DATA_DIR, "OS-1-128_v2.3.0_1024x10_lb_n3.osf"),
         os.path.join(PCAPS_DATA_DIR, 'OS-1-128_v2.3.0_1024x10_lb_n3.pcap')]

L = 3   # BOTH FILES USED ARE STRICTLY 3


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


@pytest.fixture(params=paths)
def frame_set_source_path(request):
    return request.param


def test_reduce_raises_exception_on_factors_mismatch_sensor_count(frame_set_source_path) -> None:
    normal_src = open_source(frame_set_source_path)
    normal_src = cast(FrameSetSource, normal_src)
    with pytest.raises(ValueError) as ex:
        _ = ReducedFrameSetSource(normal_src, [64] * (len(normal_src.sensor_info) + 1))
    assert str(ex.value) == "beams should match the count of sensors"


@pytest.mark.parametrize("beams", [
    (12.5),
    (20 / 2),
    (30),
])
def test_reduce_raises_exception_on_invalid_factor_values(frame_set_source_path, beams) -> None:
    normal_src = open_source(frame_set_source_path)
    normal_src = cast(FrameSetSource, normal_src)
    with pytest.raises(ValueError) as ex:
        _ = ReducedFrameSetSource(normal_src, [beams])
    assert (str(ex.value) == f"beams {beams} must be divisor of "
            f"{cast(core.LidarInfo, normal_src.sensor_info[0]).format.pixels_per_column}")


@pytest.mark.parametrize("beams", [
    (1),
    (8),
    (16),
    (32),
])
def test_reduce(frame_set_source_path, beams) -> None:
    normal_src = open_source(frame_set_source_path)
    normal_src = cast(FrameSetSource, normal_src)
    reduced_src = ReducedFrameSetSource(normal_src, [beams])
    assert cast(core.LidarInfo, reduced_src.sensor_info[0]).format.pixels_per_column == beams

    normal_frames = [s[0] for s in normal_src if s]
    reduced_frames = [s[0] for s in reduced_src if s]

    for n, r in zip(normal_frames, reduced_frames):
        nt = cast(LidarFrame, n)
        rt = cast(LidarFrame, r)
        assert rt.sensor_info == reduced_src.sensor_info[0]
        assert nt.sensor_info != rt.sensor_info
        assert nt.w == rt.w and rt.h == beams


def test_select_raises_exception_on_indices_mismatch_sensor_count(frame_set_source_path) -> None:
    normal_src = open_source(frame_set_source_path)
    normal_src = cast(FrameSetSource, normal_src)
    with pytest.raises(ValueError) as ex:
        _ = SelectedFrameSetSource(normal_src, [[0]] * (len(normal_src.sensor_info) + 1))
    assert str(ex.value) == "beam indices should match the count of sensors"


@pytest.mark.parametrize("indices, error", [
    ([0, 0], "beam indices can't contain duplicates"),
    ([-1], "beam indices [-1] must be in the range [0, 128)"),
    ([128], "beam indices [128] must be in the range [0, 128)"),
])
def test_select_raises_exception_on_invalid_indices(frame_set_source_path, indices, error) -> None:
    normal_src = open_source(frame_set_source_path)
    normal_src = cast(FrameSetSource, normal_src)
    with pytest.raises(ValueError) as ex:
        _ = SelectedFrameSetSource(normal_src, [indices])
    assert str(ex.value) == error


def test_select(frame_set_source_path) -> None:
    indices = [0, 8, 16, 32]
    normal_src = open_source(frame_set_source_path)
    normal_src = cast(FrameSetSource, normal_src)
    selected_src = SelectedFrameSetSource(normal_src, [indices])
    assert cast(core.LidarInfo, selected_src.sensor_info[0]).format.pixels_per_column == len(indices)

    normal_scans = [s[0] for s in normal_src if s]
    selected_scans = [s[0] for s in selected_src if s]

    for n, s in zip(normal_scans, selected_scans):
        nt = cast(LidarFrame, n)
        st = cast(LidarFrame, s)
        assert st.sensor_info == selected_src.sensor_info[0]
        assert nt.sensor_info != st.sensor_info
        assert nt.w == st.w and st.h == len(indices)
        for f in nt.field_types:
            if f.field_class == core.FieldClass.PIXEL_FIELD:
                assert np.array_equal(st.field(f.name), nt.field(f.name)[indices])
            else:
                assert np.array_equal(st.field(f.name), nt.field(f.name))


def test_clip_raises_exception_on_invalid_range(frame_set_source_path) -> None:
    normal_src = open_source(frame_set_source_path)
    normal_src = cast(FrameSetSource, normal_src)
    with pytest.raises(ValueError) as ex:
        _ = ClippedFrameSetSource(normal_src, [ChanField.RANGE], 5000, 1000)
    assert str(ex.value) == "`upper` value can't be less than `lower`"


@pytest.mark.parametrize("fields, lower, upper", [
    # note: selected ranges are known to work on most for most of these files
    # however, a file that has no range value which exceeds 20m would fail,
    # similarly, a file with reflectivity that is always less than 200 also fail
    # this test.
    ([ChanField.RANGE], 10000, 20000),
    ([ChanField.REFLECTIVITY], 100, 200),
])
def test_clip(frame_set_source_path, fields, lower, upper) -> None:
    normal_src = open_source(frame_set_source_path)
    normal_src = cast(FrameSetSource, normal_src)
    clipped_src = ClippedFrameSetSource(normal_src, fields, lower, upper)
    assert normal_src.sensor_info == clipped_src.sensor_info

    for s in clipped_src:
        for f in fields:
            s0 = cast(LidarFrame, s[0])
            if s0.has_field(f):
                arr = s0.field(f)
                assert np.max(arr) == upper
                nonzero = arr[arr != 0]
                assert np.min(nonzero) == lower


def test_mask_raises_exception_on_factors_mismatch_sensor_count(frame_set_source_path) -> None:
    normal_src = open_source(frame_set_source_path)
    normal_src = cast(FrameSetSource, normal_src)
    H = cast(core.LidarInfo, normal_src.sensor_info[0]).format.pixels_per_column
    W = cast(core.LidarInfo, normal_src.sensor_info[0]).format.columns_per_frame

    mask = np.vstack(
        (
            np.ones((H // 2, W), np.uint8),
            np.zeros((H // 2, W), np.uint8)
        ))

    with pytest.raises(ValueError) as ex:
        _ = MaskedFrameSetSource(normal_src, [], [mask] * (len(normal_src.sensor_info) + 1))
    assert str(ex.value) == "the number of masks should match the count of sensors"


def test_mask(frame_set_source_path) -> None:
    normal_src = open_source(frame_set_source_path)
    normal_src = cast(FrameSetSource, normal_src)
    H = cast(core.LidarInfo, normal_src.sensor_info[0]).format.pixels_per_column
    W = cast(core.LidarInfo, normal_src.sensor_info[0]).format.columns_per_frame

    mask = np.vstack(
        (
            np.ones((H // 2, W), np.uint8),
            np.zeros((H // 2, W), np.uint8)
        ))

    masked_src = MaskedFrameSetSource(normal_src, [], [mask] * len(normal_src.sensor_info))

    normal_frames = [s[0] for s in normal_src if s]
    masked_frames = [s[0] for s in masked_src if s]

    for n, m in zip(normal_frames, masked_frames):
        nt = cast(LidarFrame, n)
        mt = cast(LidarFrame, m)
        for f in nt.fields:
            assert np.max(nt.field(f)[0:H // 2, :]) == np.max(mt.field(f)[0:H // 2, :])
            assert np.max(nt.field(f)[H // 2:, :]) != 0 and np.max(mt.field(f)[H // 2:, :]) == 0


@pytest.mark.skipif(sys.platform.startswith("win"), reason="Broken on Windows")
def test_collate(input_osf_file):
    """Make sure indexing and length of collated source works correctly"""

    data = osf.OsfFrameSetSource(str(input_osf_file))
    frame = next(iter(data))[0]
    data.close()
    result = None
    f = None
    try:
        # Create an adversarial file with no overlap between the two sensors
        with tempfile.NamedTemporaryFile(delete=False) as f:
            w = osf.Writer(f.name, [frame.sensor_info, frame.sensor_info])
            for idx in range(10):
                frame.frame_id = idx
                frame.receive_timestamp[:] = 100000001 * idx + 1
                if idx >= 5:
                    w.save(1, frame)
                else:
                    w.save(0, frame)
            w.close()

        # with a dt of 200000000 we should get N-1 collated frames
        result = core.collate(osf.OsfFrameSetSource(f.name), 200000000)

        # validate the lengths are as expected
        assert result.frames_num == [5, 5]
        assert len(result) == 9

        # validate that looping through matches indexing through
        looped = []
        for frames in result:
            looped.append(copy.copy(frames))
        indexed = []
        for i in range(len(result)):
            indexed.append(copy.copy(result[i]))
        assert indexed == looped

        # with a dt of 100000000 we should get N collated frames
        result = core.collate(osf.OsfFrameSetSource(f.name), 100000000)

        # validate the lengths are as expected
        assert result.frames_num == [5, 5]
        assert len(result) == 10

        # validate that looping through matches indexing through
        looped = []
        for frames in result:
            looped.append(copy.copy(frames))
        indexed = []
        for i in range(len(result)):
            indexed.append(copy.copy(result[i]))
        assert indexed == looped
    finally:
        if result:
            result.close()
        os.unlink(f.name)


def test_chain(frame_set_source_path) -> None:
    """Make sure chaining works with frame source ops"""
    src = open_source(frame_set_source_path)
    src = src.reduce([1])
    src = src.clip(["RANGE"], 0, 100)
    src = src.mask(["RANGE"], [None])
    src = src.single(0)

    got_frame = False
    for frame in src:
        got_frame = True
        break
    assert got_frame


def test_lidar_frame_field_types_writable(frame_set_source_path) -> None:
    """Create LidarFrame, add all supported field types, verify they are writable."""
    src = open_source(frame_set_source_path)
    info = cast(core.LidarInfo, src.sensor_info[0])

    ls = LidarFrame(info)

    # Attempt to add fields of all possible numpy scalar types
    dtypes = [
        np.uint8, np.uint16, np.uint32, np.uint64,
        np.int8, np.int16, np.int32, np.int64,
        np.float32, np.float64
    ]

    for dtype in dtypes:
        try:
            ls.add_field(f"test_field_{dtype.__name__}", dtype)
        except (TypeError, ValueError):
            # Not all types might be supported by the bindings
            pass

    ls.add_field("X",
                 np.array([b'0', b'1', b'2', b'3', b'4', b'5', b'6', b'7'],
                          dtype='|S8').view(np.recarray),
                core.FieldClass.FRAME_FIELD)

    for field_name in ls.fields:
        arr = ls.field(field_name)
        # Check that it is a numpy array (or convertible/workable) and writable
        assert isinstance(arr, np.ndarray)
        assert arr.flags.writeable, f"Field {field_name} (dtype: {arr.dtype}) is not writable"


def test_filter_xyz_dewarp_points_changes_spatial_mask(frame_set_source_path) -> None:
    """Dewarping should change the XYZ mask when non-identity per-column poses are used."""
    src = cast(FrameSetSource, open_source(frame_set_source_path))
    try:
        first = next(iter(src))[0]
        assert first is not None
        frame = copy.deepcopy(cast(LidarFrame, first))
    finally:
        src.close()

    assert frame.has_field(ChanField.RANGE), "RANGE field is required for XYZ filtering test"

    frame.field(ChanField.RANGE)[:] = 1000
    frame.body_to_world[:] = np.repeat(np.eye(4, dtype=np.float64)[None, :, :], frame.w, axis=0)

    poses = np.repeat(np.eye(4, dtype=np.float64)[None, :, :], frame.w, axis=0)
    poses[:frame.w // 2, 0, 3] = 2.0

    def zero_xyz(frame_or_range: Union[LidarFrame, np.ndarray]) -> np.ndarray:
        range_image = (
            frame_or_range.field(ChanField.RANGE)
            if isinstance(frame_or_range, LidarFrame)
            else frame_or_range
        )
        h, w = range_image.shape
        return np.zeros((h, w, 3), dtype=np.float64)

    frame_body = copy.deepcopy(frame)
    frame_world = copy.deepcopy(frame)
    frame_world.body_to_world[:] = poses

    frame_ops.filter_xyz(
        frame_body,
        zero_xyz,
        axis_idx=0,
        lower=1.5,
        upper=2.5,
        invalid=0,
        filtered_fields=[ChanField.RANGE],
        dewarp_points=False,
    )
    frame_ops.filter_xyz(
        frame_world,
        zero_xyz,
        axis_idx=0,
        lower=1.5,
        upper=2.5,
        invalid=0,
        filtered_fields=[ChanField.RANGE],
        dewarp_points=True,
    )

    body_zeroed = np.count_nonzero(frame_body.field(ChanField.RANGE) == 0)
    world_zeroed = np.count_nonzero(frame_world.field(ChanField.RANGE) == 0)

    assert body_zeroed == 0
    assert world_zeroed == frame.h * (frame.w // 2)


def _make_projectable_camera_info(w=4, h=3, fx=100.0, fy=200.0, cx=1.5, cy=1.0):
    """Build a CameraInfo with pinhole intrinsics + a range scaling map.

    The shared XYZLut derives per-pixel direction/offset from the pinhole model,
    so no beam angles are stored on the CameraInfo. The returned scaling map |o|
    converts planar depth into radial RANGE.
    """
    ci = core.CameraInfo()
    ci.model = core.CameraModel(fx, fy, cx, cy)
    ci.format.columns_per_frame = w
    ci.format.pixels_per_column = h
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    x_cam = (u - cx) / fx
    y_cam = (v - cy) / fy
    z_cam = np.ones_like(x_cam)
    # ZED image space -> Ouster sensor space (X forward, Y left, Z up).
    ox, oy, oz = z_cam, -x_cam, -y_cam
    mag = np.sqrt(ox ** 2 + oy ** 2 + oz ** 2)
    return ci, mag


def _make_camera_frame(ci, mag, rng_m=2.0):
    frame = core.CameraFrame()
    frame.w = ci.format.columns_per_frame
    frame.h = ci.format.pixels_per_column
    frame.sensor_info = ci
    frame.add_field(ChanField.RANGE, np.round(rng_m * mag * 1000.0).astype(np.uint32))
    return frame


def test_valid_column_accessors_on_camera_frame() -> None:
    """get_first/last_valid_column are Frame-base methods usable on a CameraFrame."""
    ci, mag = _make_projectable_camera_info()
    cam = _make_camera_frame(ci, mag)

    # A CameraFrame has a single status entry defaulted to valid, so both
    # accessors (now hoisted onto the Frame base) return 0.
    assert cam.get_first_valid_column() == 0
    assert cam.get_last_valid_column() == 0

    # No valid columns -> both raise, exactly like a LidarFrame.
    cam.status[:] = 0
    with pytest.raises(RuntimeError):
        cam.get_first_valid_column()
    with pytest.raises(RuntimeError):
        cam.get_last_valid_column()


def test_camera_frame_generic_field_ops() -> None:
    """clip / filter_field / filter_uv operate on CameraFrame (Frame& generalization)."""
    ci, mag = _make_projectable_camera_info()

    # clip: values outside [200, 800] are zeroed, values inside are kept
    f = _make_camera_frame(ci, mag)
    f.field(ChanField.RANGE)[:] = np.arange(f.h * f.w, dtype=np.uint32).reshape(f.h, f.w) * 100
    frame_ops.clip(f, [ChanField.RANGE], 200, 800)
    vals = f.field(ChanField.RANGE).ravel()
    assert vals[0] == 0 and vals[1] == 0 and vals[2] == 200
    assert vals[8] == 800 and vals[9] == 0

    # filter_field: pixels whose RANGE is inside [200, 400] are invalidated
    g = _make_camera_frame(ci, mag)
    g.field(ChanField.RANGE)[:] = np.arange(g.h * g.w, dtype=np.uint32).reshape(g.h, g.w) * 100
    frame_ops.filter_field(g, ChanField.RANGE, 200, 400, 0)
    gvals = g.field(ChanField.RANGE).ravel()
    assert gvals[2] == 0 and gvals[3] == 0 and gvals[4] == 0
    assert gvals[1] == 100 and gvals[5] == 500

    # filter_uv 'u' zeroes rows; 'v' zeroes columns (plain mask, no destagger for cameras)
    u_frame = _make_camera_frame(ci, mag)
    u_frame.field(ChanField.RANGE)[:] = 500
    frame_ops.filter_uv(u_frame, 'u', 0, 1, 0)
    assert np.all(u_frame.field(ChanField.RANGE)[0] == 0)
    assert np.all(u_frame.field(ChanField.RANGE)[1:] == 500)

    v_frame = _make_camera_frame(ci, mag)
    v_frame.field(ChanField.RANGE)[:] = 500
    frame_ops.filter_uv(v_frame, 'v', 0, 2, 0)
    assert np.all(v_frame.field(ChanField.RANGE)[:, :2] == 0)
    assert np.all(v_frame.field(ChanField.RANGE)[:, 2:] == 500)


def test_filter_uv_3d(frame_set_source_path) -> None:
    """Test that filter uv works on 3d fields"""
    ci, mag = _make_projectable_camera_info()
    frame = _make_camera_frame(ci, mag)
    assert frame is not None
    frame.add_field("custom_float16", np.ones((frame.h, frame.w, 3), dtype=np.float16))
    frame.add_field("custom_float32", np.ones((frame.h, frame.w, 3), dtype=np.float32))
    frame.add_field("custom_float64", np.ones((frame.h, frame.w, 3), dtype=np.float64))
    frame_ops.filter_uv(frame, 'v', 0, 2, 0)
    for name in ["custom_float16", "custom_float32", "custom_float64"]:
        assert np.all(frame.field(name)[:, :2] == [0, 0, 0])
        assert np.all(frame.field(name)[:, 2:] == [1, 1, 1])


def test_camera_filter_xyz_single_pose_transform() -> None:
    """filter_xyz on a camera uses a single-pose rigid transform (no dewarp) for WORLD."""
    ci, mag = _make_projectable_camera_info()
    s2b = np.eye(4, dtype=np.float64)
    s2b[0, 3] = 1.0  # non-identity extrinsics
    ci.sensor_to_body = s2b

    lut_body = core.XYZLut(ci)

    v, u = 1, 2
    pose = np.eye(4, dtype=np.float64)
    pose[1, 3] = 50.0  # large, distinct world translation in +y

    def build_frame():
        # CameraFrame doesn't support copy.deepcopy, so build fresh frames.
        fr = _make_camera_frame(ci, mag, rng_m=0.0)
        fr.field(ChanField.RANGE)[v, u] = int(round(2.0 * mag[v, u] * 1000.0))
        fr.body_to_world[0] = pose
        return fr

    # Expected world y for the single valid pixel via the same single-pose math.
    pts_body = lut_body(build_frame().field(ChanField.RANGE))
    world = pts_body @ pose[:3, :3].T + pose[:3, 3]
    y = float(world[v, u, 1])

    # In-range WORLD y filter zeroes the valid pixel. This must not raise even
    # though body_to_world holds a single pose while w=4 (no per-column dewarp).
    in_range = build_frame()
    frame_ops.filter_xyz(in_range, lut_body, axis_idx=1, lower=y - 0.05, upper=y + 0.05,
                         invalid=0, dewarp_points=True)
    assert in_range.field(ChanField.RANGE)[v, u] == 0

    # Out-of-range WORLD y filter leaves the valid pixel untouched.
    out_range = build_frame()
    frame_ops.filter_xyz(out_range, lut_body, axis_idx=1, lower=y + 10.0, upper=y + 11.0,
                         invalid=0, dewarp_points=True)
    assert out_range.field(ChanField.RANGE)[v, u] != 0
