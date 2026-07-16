from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from typing import List, Tuple

import numpy as np
import pytest
from ouster.sdk import core, open_source


import slicing as snippet  # noqa: E402

ROOT_DIR = Path(__file__).resolve().parents[5]
OSF_PATH = ROOT_DIR / "tests" / "osfs" / "osf_fw_3.2_10scans.osf"
if not OSF_PATH.exists():
    raise FileNotFoundError(f"Required fixture missing: {OSF_PATH}")

# Deterministic frame IDs captured from the OSF above.
EXPECTED_FRAMES = [58684, 58685, 58686, 58687, 58688,
                   58689, 58690, 58691, 58692, 58693]

@contextmanager
def open_indexed_source():
    source = open_source(str(OSF_PATH), index=True, field_names=[])
    try:
        yield source
    finally:
        source.close()


def load_indexed_frame_sets() -> List[Tuple[core.LidarFrame, ...]]:
    source = open_source(str(OSF_PATH), index=True)
    try:
        frames: List[Tuple[core.LidarFrame, ...]] = []
        for frame_tuple in source:
            converted = tuple(
                core.LidarFrame(frame)
                for frame in frame_tuple
                if frame is not None
            )
            if converted:
                frames.append(converted)
    finally:
        source.close()

    frame_ids = [frame.frame_id for (frame,) in frames]
    expected_prefix = EXPECTED_FRAMES[: len(frame_ids)]
    assert (
        frame_ids == expected_prefix
    ), "OSF fixture changed; update EXPECTED_FRAMES accordingly."

    if not frames:
        raise RuntimeError(f"No frames found in {OSF_PATH}")
    return frames


def load_frame_and_info() -> Tuple[core.LidarFrame, core.SensorInfo]:
    source = open_source(str(OSF_PATH), index=True)
    try:
        frame_ptr = source[0][0]
        if frame_ptr is None:
            raise RuntimeError(f"No frames found in {OSF_PATH}")
        frame = core.LidarFrame(frame_ptr)
        info = source.sensor_info[0]
    finally:
        source.close()
    return frame, info


def extract_output(capsys) -> List[str]:
    captured = capsys.readouterr()
    return [line for line in captured.out.strip().splitlines() if line]


def test_open_indexed_source_example_opens_real_source():
    source = snippet.open_indexed_source_example(str(OSF_PATH))
    try:
        assert len(source) > 0
    finally:
        source.close()


def test_print_nth_frame_example_outputs_expected(capsys):
    source = load_indexed_frame_sets()
    snippet.print_nth_frame_example(source)
    assert extract_output(capsys) == [str(EXPECTED_FRAMES[9]), str(EXPECTED_FRAMES[9])]


def test_print_last_frame_example_outputs_expected(capsys):
    source = load_indexed_frame_sets()
    snippet.print_last_frame_example(source)
    assert extract_output(capsys) == [str(EXPECTED_FRAMES[-1])]


def test_print_first_n_frames_example_iterates_expected_frames(capsys):
    source = load_indexed_frame_sets()
    snippet.print_first_n_frames_example(source)
    expected = [str(fid) for fid in EXPECTED_FRAMES[:9]]
    assert extract_output(capsys) == expected


def test_print_last_n_frames_example_iterates_expected_frames(capsys):
    source = load_indexed_frame_sets()
    snippet.print_last_n_frames_example(source)
    expected = [str(fid) for fid in EXPECTED_FRAMES[-9:-1]]
    assert extract_output(capsys) == expected


def test_print_step_sliced_frames_example_outputs_steps(capsys):
    source = load_indexed_frame_sets()
    snippet.print_step_sliced_frames_example(source)
    expected = [str(fid) for fid in EXPECTED_FRAMES[0:10:2]]
    assert extract_output(capsys) == expected


def test_slicing_as_source_example_behaves_like_slice(capsys):
    source = load_indexed_frame_sets()
    snippet.slicing_as_source_example(source)
    output = extract_output(capsys)

    expected = ["source2 length: 5", str(EXPECTED_FRAMES[5]), str(EXPECTED_FRAMES[9])]
    expected += [str(fid) for fid in EXPECTED_FRAMES[5:10]]
    expected.append("source3 length: 2")

    assert output == expected


def test_first_valid_column_example_prints_index(capsys):
    with open_indexed_source() as source:
        expected = 0
        snippet.first_valid_column_example(source)
    assert extract_output(capsys) == [str(expected)]


def test_last_valid_column_example_prints_index(capsys):
    with open_indexed_source() as source:
        expected = 2047
        snippet.last_valid_column_example(source)
    assert extract_output(capsys) == [str(expected)]
