import pytest
import numpy as np
from ouster.sdk.core import (LidarFrame, LidarInfo, LidarMode,
                             CameraFrame, CameraInfo, CameraModel)
from ouster.cli.plugins.map_export import (_select_default_field,
                                           _all_available_fields,
                                           frame_info,
                                           frame_num_returns,
                                           frame_reference_pose,
                                           project_world_points)


def test_select_default_field_prefers_rgb():
    meta = LidarInfo.from_default(LidarMode._1024x10)
    frame = LidarFrame(meta)
    frame.add_field("RGB", np.zeros((meta.h, meta.w, 3), dtype=np.float16))
    assert frame.has_field("REFLECTIVITY")
    assert _select_default_field([frame]) == "RGB"


def test_select_default_field_falls_back_to_reflectivity():
    meta = LidarInfo.from_default(LidarMode._1024x10)
    frame = LidarFrame(meta)
    assert not frame.has_field("RGB")
    assert frame.has_field("REFLECTIVITY")
    assert _select_default_field([frame]) == "REFLECTIVITY"


def test_select_default_field_requires_field_on_all_frames():
    meta = LidarInfo.from_default(LidarMode._1024x10)
    frame_with_rgb = LidarFrame(meta)
    frame_with_rgb.add_field("RGB", np.zeros((meta.h, meta.w, 3), dtype=np.float16))
    frame_without_rgb = LidarFrame(meta)
    assert _select_default_field([frame_with_rgb, frame_without_rgb]) == "REFLECTIVITY"


def test_select_default_field_skips_none_frames():
    meta = LidarInfo.from_default(LidarMode._1024x10)
    frame = LidarFrame(meta)
    assert _select_default_field([frame, None]) == "REFLECTIVITY"
    assert _select_default_field([None]) is None
    assert _select_default_field([]) is None


def test_select_default_field_returns_none_when_no_priority_fields():
    meta = LidarInfo.from_default(LidarMode._1024x10)
    frame = LidarFrame(meta, [])
    frame.add_field("CUSTOM", np.zeros((meta.h, meta.w), dtype=np.float32))
    assert _select_default_field([frame]) is None


def test_export_dtype_keeps_native_types_and_promotes_the_rest():
    from ouster.cli.plugins.map_export import export_dtype

    for dtype in (np.uint8, np.uint16, np.uint32, np.uint64,
                  np.int8, np.int16, np.int32, np.int64,
                  np.float32, np.float64):
        assert export_dtype(dtype) == np.dtype(dtype)
    # No point cloud format has a half-float or bool property type.
    assert export_dtype(np.float16) == np.dtype(np.float32)
    assert export_dtype(np.bool_) == np.dtype(np.uint8)


def test_export_schema_matches_the_core_attribute_layout():
    from ouster.cli.plugins.map_export import AttributeField, ExportSchema

    schema = ExportSchema([
        AttributeField("RANGE", 1, np.dtype(np.uint32)),
        AttributeField("REFLECTIVITY", 1, np.dtype(np.uint8)),
        AttributeField("RGB", 3, np.dtype(np.float32)),
    ])
    # The layout is what the voxel map stores per point: the fields' own
    # types, packed without padding.
    assert schema.layout.stride == 4 + 1 + 12
    assert schema.layout.element_count == 5
    assert schema.ply_dtype.itemsize == 12 + schema.layout.stride
    # PCD packs RGB into one float, so its row is three floats shorter.
    assert schema.pcd_dtype.itemsize == 12 + 4 + 1 + 4


def test_voxel_map_round_trips_one_array_per_attribute():
    from ouster.sdk.core import VoxelHashMap, VoxelDownsampleStrategy
    from ouster.cli.plugins.map_export import AttributeField, ExportSchema

    schema = ExportSchema([
        AttributeField("RANGE", 1, np.dtype(np.uint32)),
        AttributeField("RGB", 3, np.dtype(np.float32)),
    ])
    # 4294967295 has no exact float32 representation, so it only survives if the
    # attribute really is stored and written as a uint32.
    ranges = np.array([4294967295, 16777217, 5], dtype=np.uint32)
    rgb = np.arange(9, dtype=np.float32).reshape(3, 3) / 8.0
    xyz = np.ascontiguousarray(
        np.array([[0.5, 0.5, 0.5], [10.5, 0.5, 0.5], [20.5, 0.5, 0.5]], dtype=np.float64))

    voxel_map = VoxelHashMap.create(1.0, 10, 1, VoxelDownsampleStrategy.RANDOM,
                                    schema.layout, 100.0)
    voxel_map.add_points(xyz, [ranges, rgb])
    out_xyz, attributes = voxel_map.pointcloud()

    # One array per attribute comes back, each in the type its spec declares.
    assert len(attributes) == 2
    assert attributes[0].dtype == np.uint32 and attributes[0].shape == (3,)
    assert attributes[1].dtype == np.float32 and attributes[1].shape == (3, 3)

    order = np.argsort(out_xyz[:, 0])
    assert np.array_equal(attributes[0][order], ranges)
    assert np.array_equal(attributes[1][order], rgb)


def test_voxel_map_rejects_mismatched_attribute_arrays():
    from ouster.sdk.core import VoxelHashMap, VoxelDownsampleStrategy
    from ouster.cli.plugins.map_export import AttributeField, ExportSchema

    schema = ExportSchema([AttributeField("RANGE", 1, np.dtype(np.uint32))])
    voxel_map = VoxelHashMap.create(1.0, 10, 1, VoxelDownsampleStrategy.RANDOM,
                                    schema.layout, 100.0)
    xyz = np.ascontiguousarray(np.zeros((3, 3), dtype=np.float64))

    with pytest.raises(ValueError, match="attribute layout"):
        voxel_map.add_points(xyz, [])
    # A float64 array would silently round a large uint32 -- reject it instead.
    with pytest.raises(ValueError, match="dtype uint32"):
        voxel_map.add_points(xyz, [np.zeros(3, dtype=np.float64)])
    with pytest.raises(ValueError, match="one row per point"):
        voxel_map.add_points(xyz, [np.zeros(2, dtype=np.uint32)])


def test_extremum_strategies_rank_by_attribute_index():
    from ouster.sdk.core import VoxelHashMap, VoxelDownsampleStrategy
    from ouster.cli.plugins.map_export import AttributeField, ExportSchema

    # Matches how map_export builds its layout from --field: RANGE first,
    # REFLECTIVITY second, so attribute_index 0/1 selects one or the other.
    schema = ExportSchema([
        AttributeField("RANGE", 1, np.dtype(np.uint32)),
        AttributeField("REFLECTIVITY", 1, np.dtype(np.uint8)),
    ])
    # Two points share one voxel; a third sits alone in a second voxel.
    xyz = np.ascontiguousarray(
        np.array([[0.1, 0.1, 0.1], [0.15, 0.15, 0.15], [10.5, 0.5, 0.5]], dtype=np.float64))
    ranges = np.array([100, 20, 999], dtype=np.uint32)
    reflectivity = np.array([5, 250, 9], dtype=np.uint8)

    smallest_by_range = VoxelHashMap.create(1.0, 1, 1, VoxelDownsampleStrategy.SMALLEST,
                                            schema.layout, 100.0, attribute_index=0)
    smallest_by_range.add_points(xyz, [ranges, reflectivity])
    out_xyz, (out_ranges, _) = smallest_by_range.pointcloud()
    assert sorted(out_ranges.tolist()) == [20, 999]

    largest_by_reflectivity = VoxelHashMap.create(1.0, 1, 1, VoxelDownsampleStrategy.LARGEST,
                                                  schema.layout, 100.0, attribute_index=1)
    largest_by_reflectivity.add_points(xyz, [ranges, reflectivity])
    _, (_, out_reflectivity) = largest_by_reflectivity.pointcloud()
    assert sorted(out_reflectivity.tolist()) == [9, 250]


def test_extremum_strategies_validate_attribute_index_at_construction():
    from ouster.sdk.core import VoxelHashMap, VoxelDownsampleStrategy
    from ouster.cli.plugins.map_export import AttributeField, ExportSchema

    schema = ExportSchema([AttributeField("RANGE", 1, np.dtype(np.uint32))])
    # attribute_index defaults to 0 -- RANGE, the only field -- so this is fine.
    VoxelHashMap.create(1.0, 1, 1, VoxelDownsampleStrategy.SMALLEST, schema.layout, 100.0)
    # Out of range: caught at construction, before any point is inserted, so
    # a bad --attribute-index fails map_export immediately rather than mid-export.
    with pytest.raises(ValueError, match="attribute_index"):
        VoxelHashMap.create(1.0, 1, 1, VoxelDownsampleStrategy.LARGEST, schema.layout, 100.0,
                           attribute_index=1)
    # Needs a non-empty layout -- there is nothing to rank positions-only points by.
    with pytest.raises(ValueError, match="attribute_layout"):
        VoxelHashMap.create(1.0, 1, 1, VoxelDownsampleStrategy.SMALLEST)


def test_typed_ply_carries_native_property_types(tmp_path):
    from ouster.cli.plugins.map_export import AttributeField, save_pointcloud

    fields = [
        AttributeField("RANGE", 1, np.dtype(np.uint32)),
        AttributeField("RGB", 3, np.dtype(np.float32)),
    ]
    cloud = np.array([[1.0, 2.0, 3.0, 4294967295.0, 0.25, 0.5, 0.75]], dtype=np.float64)
    output = tmp_path / "out.ply"
    save_pointcloud(str(output), cloud, fields=fields)

    with open(output, "rb") as f:
        header = b""
        while b"end_header" not in header:
            header += f.readline()
        body = f.read()
    text = header.decode("ascii")
    assert "property uint range" in text
    assert "property float red" in text

    row = np.frombuffer(body, dtype=np.dtype([
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("range", "<u4"),
        ("red", "<f4"), ("green", "<f4"), ("blue", "<f4")]))
    assert row["range"][0] == 4294967295
    assert row["x"][0] == 1.0


def _make_camera_info(w=4, h=3):
    fx = fy = 100.0
    cx = cy = 0.0
    camera_info = CameraInfo()
    camera_info.model = CameraModel(fx, fy, cx, cy)
    camera_info.format.columns_per_frame = w
    camera_info.format.pixels_per_column = h
    camera_info.sn = 42
    # The shared XYZLut derives per-pixel direction/offset from the pinhole
    # intrinsics, so no beam angles are stored on the CameraInfo.
    return camera_info


def _make_camera_frame(w=4, h=3, rng_mm=2000):
    frame = CameraFrame()
    frame.w = w
    frame.h = h
    frame.sensor_info = _make_camera_info(w, h)
    frame.add_field("RANGE", np.full((h, w), rng_mm, dtype=np.uint32))
    left = np.zeros((h, w, 3), dtype=np.uint8)
    left[:, :, 2] = 255
    frame.add_field("LEFT_BGR", left)
    return frame


def test_frame_info_dispatches_by_frame_type():
    meta = LidarInfo.from_default(LidarMode._1024x10)
    lidar_frame = LidarFrame(meta)
    assert isinstance(frame_info(lidar_frame), LidarInfo)

    camera_frame = _make_camera_frame()
    assert isinstance(frame_info(camera_frame), CameraInfo)


def test_frame_num_returns_camera_is_single():
    camera_frame = _make_camera_frame()
    assert frame_num_returns(camera_frame) == 1

    meta = LidarInfo.from_default(LidarMode._1024x10)
    lidar_frame = LidarFrame(meta)
    assert frame_num_returns(lidar_frame) == meta.num_returns


def test_frame_reference_pose_uses_camera_single_pose():
    camera_frame = _make_camera_frame()
    pose = np.eye(4)
    pose[:3, 3] = [1.0, 2.0, 3.0]
    camera_frame.body_to_world[0] = pose
    ref = frame_reference_pose(camera_frame)
    assert np.allclose(ref, pose)


def test_all_available_fields_on_camera_frame():
    camera_frame = _make_camera_frame()
    fields = _all_available_fields([camera_frame])
    assert "RANGE" in fields
    assert "LEFT_BGR" in fields


def test_project_world_points_camera_matches_body_to_world():
    camera_frame = _make_camera_frame()
    pose = np.eye(4)
    pose[:3, 3] = [1.0, 2.0, 3.0]
    camera_frame.body_to_world[0] = pose
    rng = camera_frame.field("RANGE")
    world = project_world_points(camera_frame, rng)
    assert world is not None
    body = np.asarray(camera_frame.sensor_info.xyzlut_double(rng),
                      dtype=np.float64).reshape(camera_frame.h, camera_frame.w, 3)
    expected = body @ pose[:3, :3].T + pose[:3, 3]
    np.testing.assert_allclose(world, expected, atol=1e-6)


def test_project_world_points_returns_none_without_sensor_info():
    camera_frame = _make_camera_frame()
    camera_frame.sensor_info = None
    assert project_world_points(camera_frame, camera_frame.field("RANGE")) is None


def test_select_default_field_common_across_camera_and_lidar():
    meta = LidarInfo.from_default(LidarMode._1024x10)
    lidar_frame = LidarFrame(meta)
    lidar_frame.add_field("RGB", np.zeros((meta.h, meta.w, 3), dtype=np.float16))
    camera_frame = _make_camera_frame()
    camera_frame.add_field("RGB", np.zeros((3, 4, 3), dtype=np.float16))
    # RGB is the only priority field present on both.
    assert _select_default_field([lidar_frame, camera_frame]) == "RGB"
