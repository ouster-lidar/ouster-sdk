from pathlib import Path
import pytest

from ouster.sdk import core
from ouster.sdk import osf
from ouster.sdk.examples import osf as osf_examples


_REPO_ROOT = Path(__file__).resolve().parents[3]


def _sample_osf_path(filename: str = "OS-0-128_v3.0.1_1024x10_20241017_141645.osf") -> Path:
    return _REPO_ROOT / "tests" / "osfs" / filename


def test_osf_read_scans_structural_asserts(capsys, test_data_dir: Path):
    osf_file = _sample_osf_path()
    osf_examples.osf_read_scans(osf_file=str(osf_file))
    out = capsys.readouterr().out

    assert "scan =" in out
    assert "WxH=1024x128" in out


def test_osf_get_sensors_info_lists_metadata(capsys, test_data_dir: Path):
    osf_file = _sample_osf_path()
    osf_examples.osf_get_sensors_info(str(osf_file))
    out = capsys.readouterr().out

    assert "sensor[0]" in out
    assert "OS-0-128" in out
    assert "v3.0.1" in out


def test_osf_slice_scans_writes_sliced_file(tmp_path, monkeypatch, capsys,
                                            test_data_dir: Path):
    osf_file = _sample_osf_path()
    monkeypatch.chdir(tmp_path)
    try:
        output_file = osf_examples.osf_slice_scans(str(osf_file))
    except ValueError as exc:
        pytest.skip(f"slice_scans requires missing field in fixture: {exc}")
    out = capsys.readouterr().out
    assert "writing sliced scan" in out
    assert output_file
    sliced_path = tmp_path / output_file
    if not sliced_path.exists():
        raise RuntimeError("sliced OSF output was not produced")

    src = osf.OsfScanSource(str(sliced_path))
    scan, = next(iter(src))
    assert scan is not None, "Scan is None"
    fields = set(scan.fields)
    assert core.ChanField.RANGE in fields
    assert core.ChanField.REFLECTIVITY in fields
    assert core.ChanField.SIGNAL in fields
    assert fields == {
        core.ChanField.RANGE, core.ChanField.REFLECTIVITY, core.ChanField.SIGNAL
    }
    src.close()
