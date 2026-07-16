"""
Copyright (c) 2026, Ouster, Inc.
"""

import numpy as np

import ouster.sdk.core as core
from ouster.sdk.core import Pose


def test_pose_position_and_rotation_from_list():
    pose = Pose()
    pose.position = [1.0, 2.0, 3.0]
    pose.rotation = [1.0, 0.0, 0.0, 0.0]

    np.testing.assert_allclose(pose.position, np.array([1.0, 2.0, 3.0]),
                               rtol=0, atol=1e-12)
    np.testing.assert_allclose(pose.rotation, np.array([1.0, 0.0, 0.0, 0.0]),
                               rtol=0, atol=1e-12)


def test_setting_rotation_from_list_gets_normalized():
    # Unnormalized quaternion from quaternion_pose_to_matrix test data, scaled up.
    unnormalized = np.array(
        [2.0 * 0.8446, 2.0 * 0.1913, 2.0 * 0.4619, 2.0 * 0.1913], dtype=np.float64)
    expected_rotation = unnormalized / np.linalg.norm(unnormalized)

    pose_vec = np.array([0.8446, 0.1913, 0.4619, 0.1913, 1, 2, 3],
                        dtype=np.float64)
    expected_matrix = core.quaternion_pose_to_matrix(pose_vec)

    pose = Pose()
    pose.position = [1.0, 2.0, 3.0]
    pose.rotation = unnormalized.tolist()

    np.testing.assert_allclose(pose.position, np.array([1.0, 2.0, 3.0]),
                               rtol=0, atol=1e-12)
    np.testing.assert_allclose(pose.rotation, expected_rotation, rtol=0,
                               atol=1e-12)
    assert abs(np.linalg.norm(pose.rotation) - 1.0) < 1e-12
    np.testing.assert_allclose(pose.to_matrix(), expected_matrix, rtol=0,
                               atol=1e-5)
