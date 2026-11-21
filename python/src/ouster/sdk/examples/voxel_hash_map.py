"""
Copyright (c) 2026, Ouster, Inc.
All rights reserved.

Minimal example of constructing a VoxelHashMap with a typed attribute
layout and inserting points into it.

Through its double-valued API a voxel map widens every attribute to double, which for a
RANGE (uint32) or REFLECTIVITY (uint8) value both wastes memory and, for
values wider than a double's 53-bit mantissa, loses precision. An
AttributeLayout lets the map keep each attribute in its own native type
instead. For one-shot downsampling of such a cloud without managing a map,
see voxel_downsample.py.

Run with::

    $ python -m ouster.sdk.examples.voxel_hash_map
"""

import numpy as np

from ouster.sdk.core import VoxelDownsampleStrategy, VoxelHashMap


def voxel_hash_map_example() -> None:
    # [doc-stag-attribute-map-construct]
    # attribute_layout describes the attributes that ride alongside each
    # point's xyz position: a scalar RANGE (uint32), a scalar REFLECTIVITY
    # (uint8), and a three-element NORMALS (3 x float32). Each entry is a
    # (dtype, count) pair -- the same numpy dtypes used for the arrays passed
    # to add_points() below -- so there is no need to spell out
    # AttributeSpec/AttributeType by hand. Attributes are packed back-to-back
    # with no padding, so this layout takes 4 + 1 + 3 * 4 = 17 bytes per
    # point, versus 3 * 8 = 24 bytes if all three were doubles. create()
    # picks the map implementing the strategy; voxel_size, max_distance,
    # max_points_per_voxel and min_points_per_voxel keep their usual meaning.
    voxel_map = VoxelHashMap.create(
        voxel_size=0.5, max_points_per_voxel=10, min_points_per_voxel=1,
        strategy=VoxelDownsampleStrategy.FIRST_N_POINT, attribute_layout=[
            (np.uint32, 1),   # RANGE
            (np.uint8, 1),    # REFLECTIVITY
            (np.float32, 3),  # NORMALS
        ], max_distance=100.0)
    print(f"layout stride: {voxel_map.attribute_layout.stride} bytes/point")
    # [doc-etag-attribute-map-construct]

    # [doc-stag-attribute-add-points]
    # Points go in as an Nx3 array of positions plus one array per attribute,
    # in layout order. Each array keeps its own dtype -- the map matches it
    # against the corresponding AttributeSpec instead of converting -- so a
    # value like 2**32 - 1 survives that a float or double would round. A
    # multi-element attribute like NORMALS is an (N, count) array.
    rng = np.random.default_rng(0)
    n_points = 6
    xyz = rng.uniform(-2.0, 2.0, size=(n_points, 3))
    ranges = rng.integers(0, 2**32, size=n_points, dtype=np.uint32)
    reflectivity = rng.integers(0, 256, size=n_points, dtype=np.uint8)
    normals = rng.uniform(-1.0, 1.0, size=(n_points, 3))
    normals = (normals / np.linalg.norm(normals, axis=1, keepdims=True)).astype(np.float32)

    voxel_map.add_points(xyz, [ranges, reflectivity, normals])
    # [doc-etag-attribute-add-points]

    # [doc-stag-attribute-read-back]
    # pointcloud() gives back the same shape of data: the retained positions,
    # and one array per attribute in its original type and value.
    out_xyz, [out_ranges, out_reflectivity, out_normals] = voxel_map.pointcloud()

    print(f"{out_xyz.shape[0]} points retained after voxelization")
    print(f"attribute dtypes: {out_ranges.dtype}, {out_reflectivity.dtype}, {out_normals.dtype}")
    for i in range(out_xyz.shape[0]):
        x, y, z = out_xyz[i]
        nx, ny, nz = out_normals[i]
        print(f"  xyz=({x:.2f}, {y:.2f}, {z:.2f}) "
              f"range={out_ranges[i]} reflectivity={out_reflectivity[i]} "
              f"normal=({nx:.2f}, {ny:.2f}, {nz:.2f})")
    # [doc-etag-attribute-read-back]

    # [doc-stag-attribute-read-xyz]
    # Alternatively, if only the positions matter, take .xyz: an Nx3 float64 array.
    # This is true whether or not the map has an attribute layout.
    positions = voxel_map.pointcloud().xyz
    print(f"positions only: {positions.shape} {positions.dtype}")
    # [doc-etag-attribute-read-xyz]

    # [doc-stag-attribute-read-attributes]
    # Similarly, one can also access the attributes directly via (.attributes) in
    # its original type and value as shown below.
    out_ranges, out_reflectivity, out_normals = voxel_map.pointcloud().attributes
    # [doc-etag-attribute-read-attributes]


if __name__ == "__main__":
    voxel_hash_map_example()
