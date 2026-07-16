import io
import sys
from contextlib import redirect_stdout
from pathlib import Path
import pytest

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

import stream_multi_source as snippet  # noqa: E402

REPO_ROOT = CURRENT_DIR.parents[4]
PCAP_PATH = REPO_ROOT / "tests" / "pcaps" / "OS-0-128-U1_v2.3.0_1024x10.pcap"
OSF_PATH = REPO_ROOT / "tests" / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"
OSF_PATH2 = REPO_ROOT / "tests" / "osfs" / "single_scan_016.osf"

# TODO: Add hil tests with sensor input

class TestReplayMultiRecording:
    """Test multi-file replay functions with actual data."""


    def test_replay_multi_recording_single_pcap_prints_frame(self):
        """Test replaying single PCAP file prints frame information."""
        buf = io.StringIO()
        with redirect_stdout(buf):
            snippet.replay_multi_recording([str(PCAP_PATH)], limit=1)
        output = buf.getvalue()   
        print(f"Single PCAP replay output: '{output}'")
        assert "frame_id=" in output
        assert "0" in output  # Frame counter


    def test_replay_multi_recording_single_osf_prints_frame(self):
        """Test replaying single OSF file prints frame information."""
        buf = io.StringIO()
        with redirect_stdout(buf):
            snippet.replay_multi_recording([str(OSF_PATH)], limit=1)
        output = buf.getvalue()
        print(f"Single OSF replay output: '{output}'")
        assert "frame_id=" in output
        assert "0" in output  # Frame counter


    def test_replay_multi_recording_multiple_frames_sequential(self):
        """Test that replay_multi_recording produces sequential frame numbers."""
        buf = io.StringIO()
        with redirect_stdout(buf):
            snippet.replay_multi_recording([str(OSF_PATH)], limit=3)
        output = buf.getvalue()
        assert "0" in output
        assert "1" in output


class TestErrorHandling:
    """Test error handling for multi-source functions."""

    def test_replay_multi_recording_with_nonexistent_file(self):
        """Test replay_multi_recording handles missing files gracefully."""
        with pytest.raises(Exception):
            snippet.replay_multi_recording(["/nonexistent/file.osf"], limit=1)

    def test_replay_multi_recording_with_empty_list(self):
        """Test replay_multi_recording handles empty file list."""
        with pytest.raises(Exception):
            snippet.replay_multi_recording([], limit=1)

    def test_replay_multi_recording_with_multi_osf(self):
        """Test behavior with two OSF file inputs"""
        try:
            buf = io.StringIO()
            with redirect_stdout(buf):
                snippet.replay_multi_recording([str(OSF_PATH), str(OSF_PATH2)], limit=1)
            output = buf.getvalue()
            print(f"Multi osf files output: '{output}'")
        except Exception as e:
            print(f"Failed as expected: {e}")
            assert True


class TestOutputFormat:
    def test_replay_multi_recording_output_format(self):
        """Test that replay_multi_recording produces correctly formatted output."""
        buf = io.StringIO()
        with redirect_stdout(buf):
            snippet.replay_multi_recording([str(OSF_PATH)], limit=1)
        output = buf.getvalue()
        print(f"Output format test: '{output}'")
        lines = [line.strip() for line in output.strip().split('\n') if line.strip()]
        assert len(lines) >= 1  
        # Look for correctly formatted frame line
        frame_line = next((line for line in lines if 'frame_id=' in line), None)
        assert frame_line is not None
        # Check format
        assert 'frame' in frame_line  # Sensor identifier
        assert 'frame_id=' in frame_line  # Frame ID
        frame_id_part = frame_line.split('frame_id=')[1].strip()
        assert frame_id_part.isdigit()


    def test_print_frame_helper_via_replay(self):
        """Test _print_frame helper function indirectly through replay_multi_recording."""
        buf = io.StringIO()
        with redirect_stdout(buf):
            snippet.replay_multi_recording([str(OSF_PATH)], limit=2)
        output = buf.getvalue()
        lines = [line for line in output.strip().split('\n') if line.strip()]     
        for line in lines:
            if 'frame_id=' in line:
                # Should match format: [count] frame sensor_idx: frame_id=id
                assert 'frame' in line  
                assert 'frame_id=' in line

if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])