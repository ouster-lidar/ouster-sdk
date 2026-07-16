import sys
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

import pcap_packet_source as snippet  # noqa: E402

REPO_ROOT = CURRENT_DIR.parents[4]
PCAP_PATH = REPO_ROOT / "tests" / "pcaps" / "OS-0-32-U1_v2.2.0_1024x10.pcap"


def test_count_packets_prints_and_counts(capsys):
    counts = snippet.count_packets(PCAP_PATH)

    assert counts == {"lidar": 64, "imu": 10}

    output = capsys.readouterr().out
    assert "sensor=0 lidar frame_id=1453 bytes=8448" in output
    assert "sensor=0 imu host_ts=1635444808691218000" in output
