"""Tests for the gravity alignment snippets."""
import sys
from pathlib import Path

import numpy as np

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from plumb import (  # noqa: E402
    plumb_example,
    plumb_walls_example,
)

REPO_ROOT = CURRENT_DIR.parents[5]
IMU_OSF_PATH = REPO_ROOT / "tests" / "osfs" / "OS-1-256_pitched_imu_n1.osf"


def test_plumb_example_returns_rotation():
    correction = plumb_example(str(IMU_OSF_PATH))

    assert correction.shape == (4, 4)
    assert np.isfinite(correction).all()
    assert np.allclose(correction[:3, :3] @ correction[:3, :3].T, np.eye(3))
    assert np.allclose(correction[:3, 3], 0.0)


def test_plumb_walls_example_returns_refinement():
    correction = plumb_walls_example(str(IMU_OSF_PATH))

    assert correction is not None
    assert np.isfinite(correction).all()
