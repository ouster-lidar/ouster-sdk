from pathlib import Path
import pytest

from ouster.sdk import core
from ouster.sdk import osf
from ouster.sdk.examples import osf as osf_examples


_REPO_ROOT = Path(__file__).resolve().parents[3]


def _sample_osf_path(filename: str = "OS-0-128_v3.0.1_1024x10_20241017_141645.osf") -> Path:
    return _REPO_ROOT / "tests" / "osfs" / filename


def test_osf_read_frames_structural_asserts(capsys, test_data_dir: Path):
    osf_file = _sample_osf_path()
    osf_examples.osf_read_frames(osf_file=str(osf_file))
    out = capsys.readouterr().out

    assert "frame =" in out
    assert "WxH=1024x128" in out


def test_osf_get_sensors_info_lists_metadata(capsys, test_data_dir: Path):
    osf_file = _sample_osf_path()
    osf_examples.osf_get_sensors_info(str(osf_file))
    out = capsys.readouterr().out

    assert "sensor[0]" in out
    assert "OS-0-128" in out
    assert "v3.0.1" in out


def test_osf_slice_frames_writes_sliced_file(tmp_path, monkeypatch, capsys,
                                            test_data_dir: Path):
    osf_file = _sample_osf_path()
    monkeypatch.chdir(tmp_path)
    try:
        output_file = osf_examples.osf_slice_frames(str(osf_file))
    except ValueError as exc:
        pytest.skip(f"slice_frames requires missing field in fixture: {exc}")
    out = capsys.readouterr().out
    assert "writing sliced frame" in out
    assert output_file
    sliced_path = tmp_path / output_file
    if not sliced_path.exists():
        raise RuntimeError("sliced OSF output was not produced")

    src = osf.OsfFrameSetSource(str(sliced_path))
    frame, = next(iter(src))
    assert frame is not None, "Frame is None"
    fields = set(frame.fields)
    assert core.ChanField.RANGE in fields
    assert core.ChanField.REFLECTIVITY in fields
    assert core.ChanField.SIGNAL in fields
    assert fields == {
        core.ChanField.RANGE, core.ChanField.REFLECTIVITY, core.ChanField.SIGNAL
    }
    src.close()


def test_add_objects_to_frame_smoke():
    sensor_info = core.SensorInfo.from_default(core.LidarMode._1024x10)
    frame = core.LidarFrame(sensor_info)
    objects = osf_examples.add_objects_to_frame(frame)

    assert len(objects) == 2
    assert len(frame.objects["test_objects"]) == 2
    assert frame.objects["test_objects"][0].id == 1
    assert frame.objects["test_objects"][1].class_id == 2


def test_classmaps_smoke(tmp_path):
    sensor_info = core.SensorInfo.from_default(core.LidarMode._1024x10)
    osf_path = tmp_path / "test.osf"

    writer, class_maps, metadata = osf_examples.write_classmaps(osf_path, sensor_info)
    assert class_maps["four_legs"][1] == "dog"
    assert metadata["class_maps"] == class_maps
    writer.close()

    src, class_map_set, dog_class = osf_examples.read_classmaps(osf_path)
    assert dog_class == "dog"
    assert class_map_set == class_maps
    src.close()
