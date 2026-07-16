import io
import os
import sys
import tempfile
from contextlib import redirect_stdout
from pathlib import Path

import pytest

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

import playback_osf as snippet  # noqa: E402

REPO_ROOT = CURRENT_DIR.parents[4]
OSF_FIXTURE = REPO_ROOT / "tests" / "osfs" / "single_scan_016.osf"


def capture_stdout(func, *args, **kwargs):
    buf = io.StringIO()
    with redirect_stdout(buf):
        func(*args, **kwargs)
    return buf.getvalue()


def file_size(path: Path) -> int:
    return path.stat().st_size if path.exists() else 0


def assert_osf_has_frames(path: Path) -> None:
    import ouster.sdk.osf as osf

    source = osf.OsfFrameSetSource(str(path))
    try:
        first = next(iter(source), None)
        assert first is not None, f"{path} contains no frames"
    finally:
        source.close()


def assert_metadata_matches(original: Path, derived: Path) -> None:
    import ouster.sdk.osf as osf

    original_source = osf.OsfFrameSetSource(str(original))
    derived_source = osf.OsfFrameSetSource(str(derived))

    try:
        orig_infos = list(original_source.sensor_info)
        derived_infos = list(derived_source.sensor_info)
        assert len(orig_infos) == len(derived_infos), "Sensor count mismatch"
        for base, new in zip(orig_infos, derived_infos):
            assert base.sn == new.sn
            assert base.prod_line == new.prod_line
            assert base.config == new.config
    finally:
        original_source.close()
        derived_source.close()


def test_osf_create_frames_writes_blank_frame(tmp_path: Path):
    output = tmp_path / "blank.osf"
    stdout = capture_stdout(snippet.osf_create_frames, str(output))
    assert output.exists(), "Writer should produce an OSF file."
    assert file_size(output) > 0, "OSF file should not be empty."
    assert stdout == "", f"Expected no stdout, got: {stdout!r}"

    import ouster.sdk.osf as osf

    source = osf.OsfFrameSetSource(str(output))
    try:
        iterator = iter(source)
        first = next(iterator, None)
        print("Captured frames:", first)
        assert first is not None, f"{output} contains no frames"
        frame = first[0]
        assert frame is not None, "Expected first frame to be present"
        assert frame.w > 0 and frame.h > 0, "Frame dimensions must be non-zero"
        field_types_attr = getattr(frame, "field_types")
        field_types = field_types_attr() if callable(field_types_attr) else field_types_attr
        field_names = {ft.name for ft in field_types}
        assert field_names, "Frame should contain fields"
        assert "RANGE" in field_names or "SIGNAL" in field_names, (
            "Frame should include at least RANGE or SIGNAL field"
        )
    finally:
        source.close()


def test_osf_write_frames_copies_fixture(tmp_path: Path):
    import ouster.sdk.osf as osf
    assert OSF_FIXTURE.exists(), f"Required OSF fixture is missing: {OSF_FIXTURE}"

    scratch = tmp_path / "input.osf"
    scratch.write_bytes(OSF_FIXTURE.read_bytes())

    # Monkeypatch open_source to return a deterministic iterator over the fixture.
    class StubSource:
        def __init__(self):
            self._source = osf.OsfFrameSetSource(str(scratch))
            self.sensor_info = self._source.sensor_info

        def __iter__(self):
            return iter(self._source)

        def close(self):
            self._source.close()

    import ouster.sdk as sdk  # noqa: E402
    from ouster.sdk import core
    from typing import Callable, cast

    output = tmp_path / "copied.osf"
    stdout = io.StringIO()

    original_open_source = sdk.open_source
    stub_open_source: Callable[..., core.FrameSetSource] = lambda *args, **kwargs: StubSource()  # type: ignore[assignment,return-value]
    sdk.open_source = cast(Callable[..., core.FrameSetSource], stub_open_source)
    try:
        with redirect_stdout(stdout):
            snippet.osf_write_frames("ignored", str(output), n_frames=1)
    finally:
        sdk.open_source = original_open_source

    assert output.exists(), "OSF writer should create the output file."
    assert file_size(output) > 0, "Output OSF file should contain data."
    assert stdout.getvalue() == "", f"Unexpected stdout: {stdout.getvalue()!r}"

    assert_osf_has_frames(output)
    assert_metadata_matches(scratch, output)
