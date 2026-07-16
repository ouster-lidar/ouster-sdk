"""Tests for osf.py"""
import io
import sys
from contextlib import redirect_stdout
from pathlib import Path

import pytest

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

import osf as snippet  # noqa: E402

REPO_ROOT = CURRENT_DIR.parents[4]
OSF_PATH = REPO_ROOT / "tests" / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


def validate_test_fixtures():
    """Validate that required test fixtures exist."""
    if not OSF_PATH.exists():
        pytest.fail(f"Required test fixture not found: OSF: {OSF_PATH}")


class TestOsfListLidarMetadata:
    """Test osf_list_lidar_metadata function."""

    def test_list_lidar_metadata_with_valid_osf(self, capsys):
        """Test listing lidar metadata from a valid OSF file."""
        validate_test_fixtures()
        snippet.osf_list_lidar_metadata(str(OSF_PATH))
        output = capsys.readouterr().out
        # Should find at least one metadata entry
        assert "meta[" in output
        assert "sn=" in output
        assert "fw_rev=" in output
        assert "prod_line=" in output

    def test_list_lidar_metadata_output_format(self, capsys):
        """Test that metadata output follows expected format."""
        validate_test_fixtures()
        snippet.osf_list_lidar_metadata(str(OSF_PATH))
        output = capsys.readouterr().out
        # Check that output contains expected format elements
        assert "meta[" in output or "No LidarSensor metadata entries found." in output


class TestOsfReadMessages:
    """Test osf_read_messages function."""

    def test_read_messages_with_valid_osf(self, capsys):
        """Test reading messages from a valid OSF file."""
        validate_test_fixtures()
        snippet.osf_read_messages(str(OSF_PATH))
        output = capsys.readouterr().out
        # Should print message information
        assert "message.ts:" in output
        assert "message.id:" in output

    def test_read_messages_contains_lidar_frames(self, capsys):
        """Test that reading messages can decode LidarFrame messages."""
        validate_test_fixtures()
        snippet.osf_read_messages(str(OSF_PATH))
        output = capsys.readouterr().out
        assert "message.ts:" in output
        assert "message.id:" in output
        assert "ls = " in output


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

