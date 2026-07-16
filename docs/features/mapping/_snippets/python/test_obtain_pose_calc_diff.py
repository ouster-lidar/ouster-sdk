"""Pytest coverage for pose drift snippet."""
from __future__ import annotations

import math

import numpy as np
import pytest

import pathlib
import sys

CURRENT_DIR = pathlib.Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from obtain_pose_calc_diff import pose_differences


@pytest.mark.parametrize(
    "last_pose, frame_pose, expected_translation",
    [
        (np.eye(4), np.eye(4), np.zeros(3)),
        (
            np.eye(4),
            np.array(
                [
                    [1.0, 0.0, 0.0, 1.2],
                    [0.0, 1.0, 0.0, -3.4],
                    [0.0, 0.0, 1.0, 5.6],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
            np.array([1.2, -3.4, 5.6]),
        ),
    ],
)
def test_pose_differences_translation(last_pose, frame_pose, expected_translation):
    rotation_diff, translation_diff = pose_differences(last_pose, frame_pose)
    np.testing.assert_allclose(rotation_diff, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(translation_diff, expected_translation, atol=1e-12)


def test_pose_differences_rotation():
    angle = math.pi / 4
    rot_z = np.array(
        [
            [math.cos(angle), -math.sin(angle), 0.0, 0.0],
            [math.sin(angle), math.cos(angle), 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    rotation_diff, translation_diff = pose_differences(np.eye(4), rot_z)
    expected_rotation = rot_z[:3, :3]
    np.testing.assert_allclose(rotation_diff, expected_rotation, atol=1e-12)
    np.testing.assert_allclose(translation_diff, np.zeros(3), atol=1e-12)
