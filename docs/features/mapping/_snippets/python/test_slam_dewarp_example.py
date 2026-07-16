"""Tests for the slam_dewarp_example snippet."""
import sys
from pathlib import Path

import numpy as np
import pytest

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from slam_dewarp_example import slam_dewarp_once  # noqa: E402

REPO_ROOT = CURRENT_DIR.parents[4]
OSF_PATH = REPO_ROOT / "tests" / "osfs" / "single_scan_016.osf"


def test_slam_dewarp_once_returns_points():
    cloud = slam_dewarp_once(str(OSF_PATH))
    assert cloud is not None
    assert cloud.ndim == 3
    assert cloud.shape[-1] == 3
    assert np.count_nonzero(np.abs(cloud)) > 0
