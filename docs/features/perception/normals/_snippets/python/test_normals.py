"""Tests for the normals snippet."""
import sys
from pathlib import Path

import numpy as np

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from normals import estimate_normals  # noqa: E402
from ouster.sdk import core, open_source  # noqa: E402

REPO_ROOT = CURRENT_DIR.parents[5]
OSF_PATH = REPO_ROOT / "tests" / "osfs" / "single_scan_016.osf"


def test_estimate_normals_returns_unit_vectors():
    with open_source(str(OSF_PATH)) as source:
        frame = next(iter(source))[0]
        info = source.sensor_info[0]

        range_image = frame.field(core.ChanField.RANGE)
        xyz = core.XYZLut(info)(range_image).reshape(frame.h, frame.w, 3)
        range_destaggered = core.destagger(info, range_image)
        xyz_destaggered = core.destagger(info, xyz)

    normal_image = estimate_normals(xyz_destaggered, range_destaggered)

    assert normal_image.shape == (frame.h, frame.w, 3)
    lengths = np.linalg.norm(normal_image, axis=2)
    valid = lengths > 0
    assert np.any(valid)
    assert np.allclose(lengths[valid], 1.0, atol=1e-6)
