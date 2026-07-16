"""Tests for the localization snippet."""
import io
import os
import sys
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np
import pytest

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

import localization as loc_module  # noqa: E402  (needed for monkeypatching gps_to_utm)
from localization import (  # noqa: E402
    run_localization_once,
    configure_filtering,
    gnss_initial_pose,
)

REPO_ROOT = CURRENT_DIR.parents[4]
TESTS_DIR = REPO_ROOT / "tests"

LOCALIZATION_OSF = TESTS_DIR / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"
LOCALIZATION_MAP = TESTS_DIR / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3_map-000.ply"


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def localization_source():
    if not LOCALIZATION_OSF.exists():
        raise FileNotFoundError(f"Localization test OSF not found: {LOCALIZATION_OSF}")
    return str(LOCALIZATION_OSF)


@pytest.fixture
def localization_map():
    p = Path(os.environ.get("TEST_MAP", str(LOCALIZATION_MAP)))
    if not p.exists():
        raise FileNotFoundError(
            f"Localization map not found: {p} (set TEST_MAP to override)"
        )
    return str(p)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_configure_filtering():
    """configure_filtering() should return a config with expected values."""
    config = configure_filtering()

    assert config.min_range == 1.0,  f"Expected min_range=1.0, got {config.min_range}"
    assert config.max_range == 80.0, f"Expected max_range=80.0, got {config.max_range}"


def test_gnss_initial_pose(monkeypatch):
    """gnss_initial_pose() should return a 4x4 SE(3) matrix with correct X/Y translation.

    gps_to_utm() is a user-supplied placeholder, so we inject a stub that returns
    a known UTM position and verify the resulting pose offset is correct.
    """
    # Hardcoded map origin inside the snippet (line 73 of localization.py)
    MAP_ORIGIN_UTM = (500230.0, 4182030.0)

    # Simulated sensor UTM position: 10 m east, 15 m north of the map origin
    SENSOR_UTM = (MAP_ORIGIN_UTM[0] + 10.0, MAP_ORIGIN_UTM[1] + 15.0)

    # gps_to_utm is intentionally undefined in the snippet (user-supplied placeholder),
    # so raising=False is required to inject it for the first time, otherwise it will raise an AttributeError.
    monkeypatch.setattr(loc_module, "gps_to_utm", lambda lat, lon: SENSOR_UTM, raising=False)

    pose = gnss_initial_pose(37.3861, -122.0839)

    assert pose.shape == (4, 4), f"Expected (4,4) matrix, got {pose.shape}"

    # Rotation block must be identity — GPS gives position only, no orientation
    np.testing.assert_allclose(pose[:3, :3], np.eye(3), atol=1e-9,
                               err_msg="Rotation block should be identity")

    # Bottom row must be [0, 0, 0, 1] for a valid SE(3) matrix
    np.testing.assert_allclose(pose[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9,
                               err_msg="Bottom row of SE(3) matrix is wrong")

    # X/Y translation must equal SENSOR_UTM - MAP_ORIGIN_UTM
    assert pose[0, 3] == pytest.approx(10.0), f"X offset wrong: {pose[0, 3]}"
    assert pose[1, 3] == pytest.approx(15.0), f"Y offset wrong: {pose[1, 3]}"
    assert pose[2, 3] == pytest.approx(0.0),  f"Z offset should be 0: {pose[2, 3]}"


def test_run_localization_once_prints_pose(localization_source, localization_map):
    """run_localization_once() should process all frames and print pose output."""
    buf = io.StringIO()
    with redirect_stdout(buf):
        run_localization_once(localization_source, localization_map)

    output = buf.getvalue()
    assert output, "Expected pose output to be printed but got nothing"

    lines = [l for l in output.strip().splitlines() if l.strip()]
    assert len(lines) > 0, "Expected at least one pose line"

    # Every field produced by the print() in localization.py must be present
    for line in lines:
        assert "idx ="   in line, f"Missing 'idx =' in: {line}"
        assert "ts ="    in line, f"Missing 'ts =' in: {line}"
        assert "XYZ:"    in line, f"Missing 'XYZ:' in: {line}"
        assert "R:"      in line, f"Missing 'R:' in: {line}"
        assert "P:"      in line, f"Missing 'P:' in: {line}"
        assert "Y:"      in line, f"Missing 'Y:' in: {line}"

    # Parse every line and validate numeric values against the known test dataset.
    # Dataset: OS-1-128_v2.3.0_1024x10_lb_n3.osf (3 frames, sensor barely moves near map origin)
    # Expected: idx in [1795..1797], XYZ within ~1 m of origin, angles within ~5 degrees.
    import re
    LINE_RE = re.compile(
        r"idx = (\d+); ts = (\d+); XYZ: ([-\d.]+), ([-\d.]+), ([-\d.]+) "
        r"\(R: ([-\d.]+), P: ([-\d.]+), Y: ([-\d.]+)\)"
    )

    frame_ids = []
    for line in lines:
        m = LINE_RE.search(line)
        assert m, f"Line did not match expected format: {line!r}"

        idx, ts = int(m.group(1)), int(m.group(2))
        x, y, z = float(m.group(3)), float(m.group(4)), float(m.group(5))
        roll, pitch, yaw = float(m.group(6)), float(m.group(7)), float(m.group(8))

        assert 1795 <= idx <= 1797,         f"frame_id {idx} outside expected range [1795, 1797]"
        assert ts > 0,                       f"timestamp must be positive, got {ts}"
        assert abs(x) < 1.0,                f"X={x:.3f} m unexpectedly far from map origin"
        assert abs(y) < 1.0,                f"Y={y:.3f} m unexpectedly far from map origin"
        assert abs(z) < 0.1,                f"Z={z:.3f} m unexpectedly far from map origin"
        assert abs(roll)  < 5.0,            f"Roll={roll:.1f}° unexpectedly large"
        assert abs(pitch) < 5.0,            f"Pitch={pitch:.1f}° unexpectedly large"
        assert abs(yaw)   < 5.0,            f"Yaw={yaw:.1f}° unexpectedly large"
        frame_ids.append(idx)

    assert len(frame_ids) == 3, f"Expected 3 frames from the test dataset, got {len(frame_ids)}"
    assert frame_ids == sorted(frame_ids), "Frame IDs should be in ascending order"

