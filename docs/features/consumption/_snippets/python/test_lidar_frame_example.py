"""Tests for lidar_frame_example.py"""
import io
import sys
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np
import pytest
from ouster.sdk.core import ChanField, FieldType, LidarFrame

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

import lidar_frame_example as snippet  # noqa: E402

REPO_ROOT = CURRENT_DIR.parents[4]
PCAP_PATH = REPO_ROOT / "tests" / "pcaps" / "OS-0-32-U1_v2.2.0_1024x10-single-packet.pcap"
JSON_PATH = REPO_ROOT / "tests" / "pcaps" / "OS-0-32-U1_v2.2.0_1024x10.json"


def validate_test_fixtures():
    """Validate that required test fixtures exist."""
    missing_files = []
    if not PCAP_PATH.exists():
        missing_files.append(f"PCAP: {PCAP_PATH}")
    if not JSON_PATH.exists():
        missing_files.append(f"JSON: {JSON_PATH}")
    if missing_files:
        pytest.fail("Required test fixtures not found:\n" + "\n".join(missing_files))


class TestLidarFrameConstruction:
    """Test different LidarFrame construction methods."""

    def test_create_profile_frame(self):
        """Test creating profile-based LidarFrame."""
        validate_test_fixtures()
        source, h, w = snippet.init(str(PCAP_PATH), str(JSON_PATH))
        print(f"**** Source sensor info: {source.sensor_info[0]}")
        profile_frame = snippet.create_profile_frame(source.sensor_info[0])
        print(f"3. Created profile frame ({profile_frame})")
        assert profile_frame is not None
        assert hasattr(profile_frame, 'fields')
        assert hasattr(profile_frame, 'frame_id')

    def test_create_reduced_frame(self):
        """Test creating reduced field LidarFrame."""
        source, h, w = snippet.init(str(PCAP_PATH), str(JSON_PATH))
        frame = snippet.create_reduced_frame(source.sensor_info[0])
        print(f"5. Created reduced frame ({frame})")
        assert frame is not None
        assert hasattr(frame, 'fields')
        # Should have only the reduced fields
        fields = frame.fields
        print(f"Reduced frame fields: {fields}")
        field_names = list(fields)
        assert 'RANGE' in fields, "RANGE field should be present"
        assert 'NEAR_IR' in fields, "NEAR_IR field should be present"
        assert len(fields) == 2, f"Expected only 2 fields, got {field_names}"


class TestMetadataExtraction:
    """Test metadata extraction functions."""
    def test_extract_frame_metadata(self):
        validate_test_fixtures()
        source, h, w = snippet.init(str(PCAP_PATH), str(JSON_PATH))
        frame = snippet.get_first_frame(source)
        print(f"*********Extracting metadata from frame ({frame})")
        assert frame is not None
        metadata = snippet.extract_frame_metadata(frame)
        print(f"*********Extracted metadata: {metadata}")
        assert isinstance(metadata, dict)
        assert 'frame_id' in metadata
        assert 'timestamp' in metadata
        assert 'status' in metadata
        assert 'measurement_id' in metadata

        # Validate types
        assert isinstance(metadata['frame_id'], (int, np.integer))
        assert metadata['timestamp'] is not None
        assert metadata['status'] is not None
        assert metadata['measurement_id'] is not None


class TestFieldExtraction:
    """Test field data extraction functions."""
    def test_extract_field_data(self):
        """Test extracting field data from real frame."""
        validate_test_fixtures()
        source, h, w = snippet.init(str(PCAP_PATH), str(JSON_PATH))
        frame = snippet.get_first_frame(source)
        field_data = snippet.extract_field_data(frame)
        print(f"*********Extracted field data keys: {list(field_data)}")
        assert isinstance(field_data, dict)
        assert 'range' in field_data
        assert field_data['range'] is not None
        assert isinstance(field_data['range'], np.ndarray)
        assert field_data['range'].ndim == 2
        assert field_data['range'].shape == (h, w)


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
