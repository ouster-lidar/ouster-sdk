import numpy as np
import pytest

from ouster.sdk.core import (AttributeLayout, AttributeSpec, AttributeType,
                             VoxelHashMap, VoxelDownsampleStrategy)

STRATEGIES = [VoxelDownsampleStrategy.FIRST_N_POINT, VoxelDownsampleStrategy.AVERAGE_POINT,
              VoxelDownsampleStrategy.RANDOM]


def make_map(voxel_size, max_distance, max_points_per_voxel, min_points_per_voxel,
             attribute_layout, strategy=VoxelDownsampleStrategy.FIRST_N_POINT):
    return VoxelHashMap.create(voxel_size, max_points_per_voxel, min_points_per_voxel,
                               strategy, attribute_layout, max_distance)


@pytest.mark.parametrize("strategy", STRATEGIES)
def test_numpy_tuples_match_equivalent_attribute_specs(strategy):
    from_numpy = make_map(1.0, 100.0, 10, 1, [(np.uint32, 1), (np.uint8, 1)], strategy)
    from_specs = make_map(1.0, 100.0, 10, 1, AttributeLayout([
        AttributeSpec(AttributeType.U32, 1),
        AttributeSpec(AttributeType.U8, 1),
    ]), strategy)
    assert from_numpy.attribute_layout == from_specs.attribute_layout
    assert from_numpy.attribute_layout.stride == 5


@pytest.mark.parametrize("strategy", STRATEGIES)
def test_numpy_tuples_and_attribute_specs_mix_freely(strategy):
    mixed = make_map(1.0, 100.0, 10, 1, [AttributeSpec(AttributeType.U32, 1), (np.uint8, 1)], strategy)
    all_numpy = make_map(1.0, 100.0, 10, 1, [(np.uint32, 1), (np.uint8, 1)], strategy)
    assert mixed.attribute_layout == all_numpy.attribute_layout


@pytest.mark.parametrize("strategy", STRATEGIES)
def test_multi_element_numpy_attribute(strategy):
    voxel_map = make_map(1.0, 100.0, 10, 1, [(np.float32, 3)], strategy)
    assert voxel_map.attribute_layout.stride == 12
    assert voxel_map.attribute_layout.element_count == 3


@pytest.mark.parametrize("dtype_like", [np.uint32, np.dtype(np.uint32), np.dtype("uint32"), "uint32"])
def test_numpy_tuples_accept_any_dtype_spelling(dtype_like):
    from_spelling = make_map(1.0, 100.0, 10, 1, [(dtype_like, 1)])
    canonical = make_map(1.0, 100.0, 10, 1, [(np.uint32, 1)])
    assert from_spelling.attribute_layout == canonical.attribute_layout


def test_unsupported_dtype_raises():
    with pytest.raises(ValueError):
        make_map(1.0, 100.0, 10, 1, [(np.complex64, 1)])


def test_malformed_entry_raises():
    with pytest.raises(ValueError):
        make_map(1.0, 100.0, 10, 1, [(np.uint32, 1, "extra")])
    with pytest.raises(ValueError):
        make_map(1.0, 100.0, 10, 1, [np.uint32])


def test_numpy_layout_usable_end_to_end():
    voxel_map = make_map(
        voxel_size=0.5, max_distance=100.0, max_points_per_voxel=10,
        min_points_per_voxel=1, attribute_layout=[(np.uint32, 1), (np.uint8, 1)])

    xyz = np.zeros((2, 3))
    ranges = np.array([10, 20], dtype=np.uint32)
    reflectivity = np.array([1, 2], dtype=np.uint8)
    voxel_map.add_points(xyz, [ranges, reflectivity])

    out_xyz, (out_ranges, out_reflectivity) = voxel_map.pointcloud()
    assert out_xyz.shape[0] == 1
    assert out_ranges.dtype == np.uint32
    assert out_reflectivity.dtype == np.uint8


def test_f16_attribute_usable_end_to_end():
    # np.float16 is a supported attribute dtype, same as the wider floats.
    voxel_map = make_map(
        voxel_size=0.5, max_distance=100.0, max_points_per_voxel=10,
        min_points_per_voxel=1, attribute_layout=[(np.float16, 1), (np.uint8, 1)])
    assert voxel_map.attribute_layout.stride == 2 + 1

    xyz = np.array([[0.1, 0.1, 0.1], [5.1, 5.1, 5.1]])
    signal = np.array([1.5, -0.25], dtype=np.float16)
    tag = np.array([3, 200], dtype=np.uint8)
    voxel_map.add_points(xyz, [signal, tag])

    out_xyz, (out_signal, out_tag) = voxel_map.pointcloud()
    assert out_xyz.shape == (2, 3)
    assert out_signal.dtype == np.float16
    assert out_tag.dtype == np.uint8
    for i in range(2):
        row = int(np.argmin(np.sum((out_xyz - xyz[i]) ** 2, axis=1)))
        assert out_signal[row] == signal[i]
        assert out_tag[row] == tag[i]

    assert AttributeType.F16.dtype == np.dtype(np.float16)
    assert AttributeLayout([(np.float16, 1)]) == AttributeLayout(
        [AttributeSpec(AttributeType.F16, 1)])


def test_add_points_with_attributes_spans_multiple_packing_blocks():
    # More rows than the binding packs per block (4096), each in its own voxel,
    # so every record has to survive the block boundaries intact.
    n_points = 10_000
    voxel_map = make_map(
        voxel_size=1.0, max_distance=1e9, max_points_per_voxel=1,
        min_points_per_voxel=1, attribute_layout=[(np.uint32, 1), (np.float32, 3)])

    xyz = np.zeros((n_points, 3))
    xyz[:, 0] = np.arange(n_points) + 0.5
    ranges = np.arange(n_points, dtype=np.uint32) * 7 + 3
    rgb = np.stack([np.arange(n_points, dtype=np.float32) + k for k in range(3)], axis=1)
    voxel_map.add_points(xyz, [ranges, rgb])

    out_xyz, (out_ranges, out_rgb) = voxel_map.pointcloud()
    order = np.argsort(out_xyz[:, 0])
    np.testing.assert_array_equal(out_xyz[order], xyz)
    np.testing.assert_array_equal(out_ranges[order], ranges)
    np.testing.assert_array_equal(out_rgb[order], rgb)


def test_add_points_with_attributes_rejects_bad_arrays_without_partial_insert():
    voxel_map = make_map(1.0, 1e9, 1, 1, [(np.uint32, 1), (np.uint8, 1)])
    xyz = np.zeros((5000, 3))
    xyz[:, 0] = np.arange(5000) + 0.5
    with pytest.raises(ValueError):
        voxel_map.add_points(
            xyz, [np.zeros(5000, dtype=np.uint32), np.zeros(5000, dtype=np.uint16)])
    assert voxel_map.empty


def test_attribute_type_dtype_round_trips_for_every_type():
    # The dtype mapping comes from the C++ scalar types; every AttributeType
    # has a distinct numpy dtype, and from_dtype inverts it for any spelling.
    dtypes = [attribute_type.dtype for attribute_type in AttributeType]
    assert len(set(dtypes)) == len(AttributeType)
    for attribute_type in AttributeType:
        assert isinstance(attribute_type.dtype, np.dtype)
        assert AttributeType.from_dtype(attribute_type.dtype) == attribute_type
        assert AttributeType.from_dtype(attribute_type.dtype.name) == attribute_type
        assert AttributeType.from_dtype(attribute_type.dtype.type) == attribute_type
    assert AttributeType.U32.dtype == np.dtype(np.uint32)
    assert AttributeType.F16.dtype == np.dtype(np.float16)
    assert AttributeType.F64.dtype == np.dtype(np.float64)


@pytest.mark.parametrize("dtype", [np.complex128, np.bool_, np.complex64])
def test_attribute_type_from_dtype_rejects_unsupported_dtypes(dtype):
    with pytest.raises(ValueError, match="expected one of: uint8"):
        AttributeType.from_dtype(dtype)


def test_attribute_layout_accepts_dtype_pairs():
    from_pairs = AttributeLayout([(np.uint32, 1), ("float32", 3)])
    from_specs = AttributeLayout([AttributeSpec(AttributeType.U32, 1),
                                  AttributeSpec(AttributeType.F32, 3)])
    assert from_pairs == from_specs
    assert from_pairs.stride == 4 + 12


def test_attribute_dtype_mismatch_names_the_numpy_dtype():
    voxel_map = make_map(1.0, 100.0, 10, 1, [(np.uint32, 1)])
    with pytest.raises(ValueError, match="dtype uint32"):
        voxel_map.add_points(np.zeros((1, 3)), [np.zeros(1, dtype=np.int32)])


def test_attribute_spec_repr_uses_the_short_type_name():
    assert repr(AttributeSpec(AttributeType.U32, 1)) == "AttributeSpec(u32, 1)"
    assert repr(AttributeSpec(AttributeType.F64, 3)) == "AttributeSpec(f64, 3)"


def test_attribute_layout_byte_offset_is_bounds_checked():
    layout = AttributeLayout([(np.uint32, 1), (np.uint8, 1)])
    assert layout.byte_offset(0) == 0
    assert layout.byte_offset(1) == 4
    for bad in (2, 1_000_000):
        with pytest.raises(IndexError, match="out of range"):
            layout.byte_offset(bad)
    with pytest.raises(IndexError):
        AttributeLayout().byte_offset(0)


def test_attribute_layout_hash_is_consistent_with_eq():
    a = AttributeLayout([(np.uint32, 1), (np.float32, 3)])
    b = AttributeLayout([AttributeSpec(AttributeType.U32, 1), AttributeSpec(AttributeType.F32, 3)])
    assert a == b and hash(a) == hash(b)
    assert len({a, b}) == 1
    assert {a: "x"}[b] == "x"
    # Order, type and count all matter.
    assert len({a,
                AttributeLayout([(np.float32, 3), (np.uint32, 1)]),
                AttributeLayout([(np.uint32, 1), (np.float32, 2)]),
                AttributeLayout([(np.int32, 1), (np.float32, 3)]),
                AttributeLayout()}) == 5
    assert hash(AttributeLayout()) == hash(AttributeLayout([]))
    # A layout can't be changed through `specs`, so its hash is stable.
    before = hash(a)
    a.specs[0].count = 7
    assert a.specs[0].count == 1 and hash(a) == before


def test_attribute_layout_specs_are_copies():
    # Mutating a returned spec must not desynchronize the layout from its
    # stride/offsets (a stale stride would make packing overrun its records).
    layout = AttributeLayout([(np.uint32, 1), (np.uint8, 1)])
    specs = layout.specs
    specs[0].count = 7
    specs[1].type = AttributeType.F64
    assert [(s.type, s.count) for s in layout.specs] == [
        (AttributeType.U32, 1), (AttributeType.U8, 1)]
    assert layout.stride == 5 and layout.element_count == 2
