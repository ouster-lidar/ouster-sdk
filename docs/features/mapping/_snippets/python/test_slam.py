"""Tests for the slam snippet."""
import io
import sys
from contextlib import redirect_stdout
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from slam import run_slam_once  # noqa: E402

REPO_ROOT = CURRENT_DIR.parents[4]
OSF_PATH = REPO_ROOT / "tests" / "osfs" / "single_scan_016.osf"


def test_run_slam_once_prints_pose():
    buf = io.StringIO()
    with redirect_stdout(buf):
        run_slam_once(str(OSF_PATH))
    output = buf.getvalue()
    assert "idx =" in output
    assert "Translation:" in output
    assert "Roll:" in output
    assert "Pitch:" in output
    assert "Yaw:" in output
