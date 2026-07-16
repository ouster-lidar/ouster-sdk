import os
import sys
from pathlib import Path

import pytest

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

import read_packet as snippet  # noqa: E402


def test_read_pcap_open_packet_source_live(capsys):
    hostname = os.getenv("SENSOR_HOSTNAME")
    if not hostname:
        pytest.skip("Set SENSOR_HOSTNAME to run live sensor packet test")
    snippet.read_pcap_open_packet_source(
        hostname,
        n_seconds=1,
        preview_packets=1,
    )
    output = capsys.readouterr().out
    assert "Packet" in output
    assert "type=" in output
    assert "bytes=" in output
    assert "host_ts=" in output
    assert "Captured" in output
