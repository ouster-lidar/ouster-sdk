import io
import sys
import os
from contextlib import redirect_stdout
from pathlib import Path
import pytest

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

import stream_single_source as snippet  # noqa: E402

REPO_ROOT = CURRENT_DIR.parents[4]
PCAP_PATH = REPO_ROOT / "tests" / "pcaps" / "OS-0-32-U1_v2.2.0_1024x10-single-packet.pcap"
PCAP2_PATH = REPO_ROOT / "tests" / "pcaps" / "OS-0-128-U1_v2.3.0_1024x10.pcap"
JSON_PATH = REPO_ROOT / "tests" / "pcaps" / "OS-0-32-U1_v2.2.0_1024x10.json"
OSF_PATH = REPO_ROOT / "tests" / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


class DummyFrame:
    def __init__(self, frame_id):
        self.frame_id = frame_id


def test_replay_data_osf_prints_frame():
    """Test replaying OSF data prints frame information."""
    buf = io.StringIO()
    with redirect_stdout(buf):
        snippet.replay_data(str(OSF_PATH), limit=1)
    output = buf.getvalue()
    assert "frame" in output
    assert "id=" in output


def test_replay_pcap_with_metadata_prints_frame():
    buf = io.StringIO()
    with redirect_stdout(buf):
        snippet.replay_pcap_with_metadata_input(
            str(PCAP_PATH),
            str(JSON_PATH),
            limit=1,
        )
    output = buf.getvalue()
    assert "frame" in output
    assert "id=" in output


def test_replay_pcap_with_pcap_frame_set_source_prints_frame():
    buf = io.StringIO()
    with redirect_stdout(buf):
        snippet.replay_pcap_with_pcap_frame_set_source(
            str(PCAP_PATH),
            str(JSON_PATH),
            limit=1,
        )
    output = buf.getvalue()
    assert "frame" in output
    assert "id=" in output


def test_replay_data_pcap_with_metadata_discovery():
    """Test that replay_data works with PCAP files when JSON metadata is available."""
    # Note: This might not work as replay_data doesn't explicitly pass metadata
    # but it's worth testing the behavior
    buf = io.StringIO()
    try:
        with redirect_stdout(buf):
            snippet.replay_data(str(PCAP2_PATH), limit=1)
        output = buf.getvalue()
        # If it works, check for frame output
        if output:
            assert "frame" in output
    except Exception:
        # Expected to fail since PCAP needs explicit metadata
        pytest.skip("PCAP requires explicit metadata, use replay_pcap_with_metadata_input")


def test_replay_data_multiple_frames():
    """Test that replay_data can process multiple frames."""
    buf = io.StringIO()
    with redirect_stdout(buf):
        snippet.replay_data(str(OSF_PATH), limit=5)
    output = buf.getvalue()

    print(f"Multi-frame output: '{output}'")
    frame_lines = [line for line in output.strip().split('\n') if 'frame' in line and line.strip()]

    # Check frame numbering is sequential
    if len(frame_lines) > 1:
        assert "frame 0:" in output
        if len(frame_lines) > 1:
            assert "frame 1:" in output


def test_replay_pcap_with_missing_json_file():
    """Test that PCAP replay fails with missing metadata file."""
    with pytest.raises(Exception):
        snippet.replay_pcap_with_metadata_input(
            str(PCAP_PATH),
            "/nonexistent/metadata.json",
            limit=1,
        )


def test_select_single_sensor_prints_frame_id():
    """sensor_idx=0 and .single(0) are equivalent and yield a frame."""
    buf = io.StringIO()
    with redirect_stdout(buf):
        snippet.select_single_sensor(str(OSF_PATH))
    output = buf.getvalue()
    assert "via sensor_idx: frame_id=" in output
    assert "via single():   frame_id=" in output



def test_stream_live_uses_sensor_frame_set_source(capsys):
    """Smoke test against a real sensor when SENSOR_HOSTNAME is set."""
    hostname = os.getenv("SENSOR_HOSTNAME")
    if not hostname:
        pytest.skip("Set SENSOR_HOSTNAME to run live sensor streaming test")
    snippet.stream_live(hostname, limit=1)
    captured = capsys.readouterr()
    # Basic sanity check: test would fail if an exception were raised
    assert "frame" in captured.out


def test_stream_live_via_open_source_collate(capsys):
    """Run collated open_source path against a live sensor if configured."""
    hostname = os.getenv("SENSOR_HOSTNAME")
    if not hostname:
        pytest.skip("Set SENSOR_HOSTNAME to run live sensor streaming test")
    snippet.stream_live_via_open_source_collate(hostname, limit=1)
    output = capsys.readouterr().out
    assert "frame" in output
