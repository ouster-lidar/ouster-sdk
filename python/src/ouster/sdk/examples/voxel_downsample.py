"""
Copyright (c) 2026, Ouster, Inc.
All rights reserved.

Minimal example of downsampling a pointcloud whose attributes keep their own
types.

voxel_downsample() takes and returns one numpy array per attribute riding
along with xyz -- a RANGE (uint32), a REFLECTIVITY (uint8), a NORMALS vector
(3 x float32) -- each in its own dtype, rather than widening them all into a
single double array. It speaks the same shape of data as
VoxelHashMap.add_points() (the ``(xyz, attributes)`` overload) / pointcloud() (see
voxel_hash_map.py).

Run with::

    $ python -m ouster.sdk.examples.voxel_downsample
"""

import numpy as np

from ouster.sdk.core import VoxelDownsampleStrategy, voxel_downsample


def voxel_downsample_example() -> None:
    # [doc-stag-attribute-downsample-input]
    # A synthetic cloud: positions plus three attributes, each in the type a
    # sensor would produce it in. A scalar attribute is a (N,) array, a
    # multi-element one (N, K) -- here NORMALS with K = 3.
    rng = np.random.default_rng(0)
    n_points = 1000
    xyz = rng.uniform(-5.0, 5.0, size=(n_points, 3))
    ranges = rng.integers(0, 2**32, size=n_points, dtype=np.uint32)
    reflectivity = rng.integers(0, 256, size=n_points, dtype=np.uint8)
    normals = rng.uniform(-1.0, 1.0, size=(n_points, 3))
    normals = (normals / np.linalg.norm(normals, axis=1, keepdims=True)).astype(np.float32)
    # [doc-etag-attribute-downsample-input]

    # [doc-stag-attribute-downsample]
    # No layout needs to be declared: each array's dtype and width define it.
    # voxel_size is the voxel edge length; max_points_per_voxel,
    # min_points_per_voxel and strategy select how each voxel is reduced, in the
    # same order as VoxelHashMap.create(). Leaving attributes out downsamples
    # positions only.
    out_xyz, [out_ranges, out_reflectivity, out_normals] = voxel_downsample(
        xyz, voxel_size=1.0, max_points_per_voxel=1, min_points_per_voxel=1,
        strategy=VoxelDownsampleStrategy.RANDOM, attributes=[ranges, reflectivity, normals])

    print(f"{n_points} points downsampled to {out_xyz.shape[0]}")
    print(f"attribute dtypes: {out_ranges.dtype}, {out_reflectivity.dtype}, {out_normals.dtype}")
    print(f"attribute shapes: {out_ranges.shape}, {out_reflectivity.shape}, {out_normals.shape}")

    # RANDOM (like FIRST_N_POINT) keeps actual input points, so each kept
    # point's attributes are exactly those it was given -- no rounding
    # through a double. (Positions, by contrast, may come back within a tiny
    # fraction of the voxel size of the input: the map's default AUTO storage
    # mode keeps them in fixed point. Hence finding the source point by
    # nearest position rather than exact equality.)
    kept = int(np.argmin(np.linalg.norm(xyz - out_xyz[0], axis=1)))
    assert out_ranges[0] == ranges[kept]
    assert out_reflectivity[0] == reflectivity[kept]
    assert np.array_equal(out_normals[0], normals[kept])
    # [doc-etag-attribute-downsample]

    # [doc-stag-attribute-downsample-average]
    # AVERAGE_POINT computes each voxel's mean. Float attributes keep the
    # exact mean; integer attributes come back in their own type with the
    # mean rounded to the nearest representable value.
    avg_xyz, [avg_ranges, avg_reflectivity, avg_normals] = voxel_downsample(
        xyz, voxel_size=1.0, strategy=VoxelDownsampleStrategy.AVERAGE_POINT,
        attributes=[ranges, reflectivity, normals])

    print(f"AVERAGE_POINT: {avg_xyz.shape[0]} voxels, "
          f"dtypes {avg_ranges.dtype}, {avg_reflectivity.dtype}, {avg_normals.dtype}")
    for i in range(min(3, avg_xyz.shape[0])):
        x, y, z = avg_xyz[i]
        nx, ny, nz = avg_normals[i]
        print(f"  xyz=({x:.2f}, {y:.2f}, {z:.2f}) range={avg_ranges[i]} "
              f"reflectivity={avg_reflectivity[i]} normal=({nx:.2f}, {ny:.2f}, {nz:.2f})")
    # [doc-etag-attribute-downsample-average]


if __name__ == "__main__":
    voxel_downsample_example()
