import numpy as np

from ouster.sdk.core import (VoxelHashMap, VoxelDownsampleStrategy,
                             VHMInternalStorageMode)
from ouster.sdk.mapping import AdaptiveThreshold, ICPRegistration


def test_registration_defaults():
    reg = ICPRegistration()
    assert reg.max_num_iterations == 50
    assert reg.convergence_criterion == 0.0001
    assert reg.max_num_threads > 0


def test_registration_align_points_to_map_identity_on_empty_map():
    reg = ICPRegistration(max_num_iterations=5)
    frame = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float64)
    voxel_map = VoxelHashMap.create(
        0.5, 20, strategy=VoxelDownsampleStrategy.FIRST_N_POINT, max_distance=10.0)

    transform = reg.align_points_to_map(frame, voxel_map, max_distance=1.0, kernel_scale=0.1)

    assert transform.shape == (4, 4)
    np.testing.assert_allclose(transform, np.eye(4), atol=1e-12)


def test_registration_align_points_to_map_with_map():
    reg = ICPRegistration(max_num_iterations=20)
    map_points = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    voxel_map = VoxelHashMap.create(
        0.5, 20, strategy=VoxelDownsampleStrategy.FIRST_N_POINT, max_distance=10.0)
    voxel_map.add_points(map_points)

    shifted_frame = map_points + np.array([0.05, 0.02, -0.01])
    transform = reg.align_points_to_map(
        shifted_frame, voxel_map, max_distance=0.5, kernel_scale=0.1
    )

    assert transform.shape == (4, 4)
    recovered = (transform[:3, :3] @ shifted_frame.T).T + transform[:3, 3]
    np.testing.assert_allclose(recovered, map_points, atol=0.05)


def test_registration_align_points_to_map_with_attributed_voxel_map():
    reg = ICPRegistration(max_num_iterations=20)
    map_points = np.array(
        [
            [0.0, 0.0, 0.0, 10.0],
            [1.0, 0.0, 0.0, 20.0],
            [0.0, 1.0, 0.0, 30.0],
            [0.0, 0.0, 1.0, 40.0],
        ],
        dtype=np.float64,
    )
    voxel_map = VoxelHashMap.create(
        0.5, 20, strategy=VoxelDownsampleStrategy.FIRST_N_POINT,
        attribute_layout=[(np.float64, 1)], max_distance=10.0)
    voxel_map.add_points(map_points)

    shifted_frame = map_points[:, :3] + np.array([0.05, 0.02, -0.01])
    transform = reg.align_points_to_map(
        shifted_frame, voxel_map, max_distance=0.5, kernel_scale=0.1
    )

    assert transform.shape == (4, 4)
    recovered = (transform[:3, :3] @ shifted_frame.T).T + transform[:3, 3]
    np.testing.assert_allclose(recovered, map_points[:, :3], atol=0.05)


def test_random_voxel_map_keeps_up_to_max_points_per_voxel():
    # All points fall in the same voxel; reservoir sampling caps the output
    # at max_points_per_voxel.
    voxel_map = VoxelHashMap.create(
        voxel_size=1.0, max_points_per_voxel=2, strategy=VoxelDownsampleStrategy.RANDOM,
        internal_storage_mode=VHMInternalStorageMode.ABS)
    points = np.array(
        [
            [0.1, 0.1, 0.1],
            [0.2, 0.2, 0.2],
            [0.3, 0.3, 0.3],
            [0.4, 0.4, 0.4],
        ],
        dtype=np.float64,
    )
    voxel_map.add_points(points)

    cloud, attributes = voxel_map.pointcloud()
    assert cloud.shape == (2, 3) and attributes == []
    # Every surviving point must be one of the inserted points.
    for row in cloud:
        assert np.any(np.all(np.isclose(points, row), axis=1))


def test_random_attributed_voxel_map_preserves_attributes():
    voxel_map = VoxelHashMap.create(
        voxel_size=1.0, max_points_per_voxel=1, strategy=VoxelDownsampleStrategy.RANDOM,
        attribute_layout=[(np.float64, 1)], internal_storage_mode=VHMInternalStorageMode.ABS)
    points = np.array(
        [
            [0.1, 0.1, 0.1, 10.0],
            [5.1, 5.1, 5.1, 20.0],
        ],
        dtype=np.float64,
    )
    voxel_map.add_points(points)

    xyz, (values,) = voxel_map.pointcloud()
    cloud = np.column_stack([xyz, values])
    # Two distinct voxels, one point each, attributes carried through.
    assert cloud.shape == (2, 4)
    for row in cloud:
        assert np.any(np.all(np.isclose(points, row), axis=1))


def test_adaptive_threshold_defaults():
    threshold = AdaptiveThreshold(max_range=100.0)
    assert threshold.max_range == 100.0
    assert threshold.min_motion_threshold == 0.01
    assert threshold.compute_threshold() == 2.0


def test_adaptive_threshold_update_and_compute():
    threshold = AdaptiveThreshold(max_range=100.0, initial_threshold=1.0)
    deviation = np.eye(4)
    deviation[:3, 3] = [2.0, 0.0, 0.0]
    threshold.update_model_deviation(deviation)
    assert threshold.compute_threshold() > 1.0
