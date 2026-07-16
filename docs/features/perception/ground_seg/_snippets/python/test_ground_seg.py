"""Tests for the ground segmentation snippet."""
import sys
from pathlib import Path

import numpy as np

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from ground_seg import segment_ground  # noqa: E402
from ouster.sdk import open_source  # noqa: E402

REPO_ROOT = CURRENT_DIR.parents[5]
OSF_PATH = REPO_ROOT / "tests" / "osfs" / "single_scan_016.osf"


def test_segment_ground_returns_binary_mask():
    with open_source(str(OSF_PATH)) as source:
        plumb_extrinsics = [
            np.asarray(info.sensor_to_body) for info in source.sensor_info
        ]
        h = source.sensor_info[0].format.pixels_per_column
        w = source.sensor_info[0].format.columns_per_frame

    ground_mask = segment_ground(str(OSF_PATH), plumb_extrinsics)

    assert ground_mask.shape == (h, w)
    assert ground_mask.dtype == np.uint8
    assert set(np.unique(ground_mask)) <= {0, 1}
    assert np.any(ground_mask)
