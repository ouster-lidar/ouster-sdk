"""Tests for the point cloud alignment snippets."""
import sys
from pathlib import Path

import numpy as np

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from align_clouds import (  # noqa: E402
    align_pairwise_example,
    align_frame_set_example,
)

REPO_ROOT = CURRENT_DIR.parents[5]
OSF_PATH = REPO_ROOT / "tests" / "osfs" / "pose_delta_2_128.osf"


def test_align_pairwise_example_returns_transform():
    transform = align_pairwise_example(str(OSF_PATH))

    assert transform.shape == (4, 4)
    assert np.isfinite(transform).all()
    assert np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0])


def test_align_frame_set_example_returns_extrinsics():
    extrinsics = align_frame_set_example(str(OSF_PATH))

    assert len(extrinsics) == 2
    assert all(extrinsic.shape == (4, 4) for extrinsic in extrinsics)
    assert all(np.isfinite(extrinsic).all() for extrinsic in extrinsics)
