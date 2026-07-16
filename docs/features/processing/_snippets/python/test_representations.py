"""Tests for the representations transform snippet."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

import representations as snippet  # noqa: E402
from ouster.sdk import core  # noqa: E402

REPO_ROOT = CURRENT_DIR.parents[4]
PCAP_PATH = REPO_ROOT / "tests" / "pcaps" / "OS-0-32-U1_v2.2.0_1024x10-single-packet.pcap"
JSON_PATH = REPO_ROOT / "tests" / "pcaps" / "OS-0-32-U1_v2.2.0_1024x10.json"


def _ensure_fixtures_exist() -> None:
    missing = [str(path) for path in (PCAP_PATH, JSON_PATH) if not path.exists()]
    if missing:
        raise RuntimeError("Required test fixtures are missing:\n" + "\n".join(missing))


@pytest.fixture(scope="module")
def frame_data():
    _ensure_fixtures_exist()
    return snippet.load_first_frame(str(PCAP_PATH), str(JSON_PATH))


def test_apply_xyz_transform_shapes(frame_data):
    sensor_info, frame = frame_data
    adjusted = snippet.apply_xyz_transform(sensor_info, frame)
    h = sensor_info.format.pixels_per_column
    w = sensor_info.format.columns_per_frame
    assert adjusted.shape == (h, w, 3)
    assert np.isfinite(adjusted).all()


def test_apply_xyz_transform_differs_from_default(frame_data):
    sensor_info, frame = frame_data
    default_xyz = core.XYZLut(sensor_info, use_extrinsics=False)(frame)
    adjusted_xyz = snippet.apply_xyz_transform(sensor_info, frame)
    assert not np.allclose(default_xyz, adjusted_xyz)


def test_apply_xyz_transform_with_extrinsics(frame_data):
    """Mirror the C++ convenience make_xyz_lut(info, true) snippet."""
    sensor_info, frame = frame_data
    extrinsic_xyz = snippet.apply_xyz_transform_w_extrinsics(sensor_info, frame)
    direct_xyz = core.XYZLut(sensor_info, use_extrinsics=True)(frame)
    assert np.allclose(extrinsic_xyz, direct_xyz)


def test_reflectivity_images_destagger(frame_data):
    sensor_info, frame = frame_data
    reflectivity, reflectivity_destaggered = snippet.reflectivity_images(sensor_info, frame)
    assert reflectivity.shape == reflectivity_destaggered.shape
    if np.any(reflectivity):
        assert not np.array_equal(reflectivity, reflectivity_destaggered)


def test_x_image_destagger(frame_data):
    sensor_info, frame = frame_data
    x_staggered = snippet.get_x_in_image_form(frame, sensor_info, destaggered=False)
    x_destaggered = snippet.get_x_in_image_form(frame, sensor_info, destaggered=True)
    assert x_staggered.shape == x_destaggered.shape
    if np.any(x_staggered):
        assert not np.array_equal(x_staggered, x_destaggered)


def test_apply_xyzf_transform_matches_manual(frame_data):
    sensor_info, frame = frame_data
    manual = snippet.apply_xyz_transform(sensor_info, frame)
    pre_lut = snippet.apply_xyzf_transform(sensor_info, frame)
    assert np.allclose(manual, pre_lut, atol=1e-3)


def test_apply_xyz_transform_via_metadata_matches_manual(frame_data):
    sensor_info, frame = frame_data
    manual = snippet.apply_xyz_transform(sensor_info, frame)
    via_metadata = snippet.apply_xyz_transform_via_metadata(
        str(PCAP_PATH), str(JSON_PATH))
    assert via_metadata.shape == manual.shape
    assert np.allclose(via_metadata, manual, atol=1e-3)


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
