from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from typing import List, Tuple, cast

import numpy as np
import pytest
from ouster.sdk import core, open_source


import slicing as snippet  # noqa: E402

ROOT_DIR = Path(__file__).resolve().parents[5]
OSF_PATH = ROOT_DIR / "tests" / "osfs" / "osf_fw_3.2_10scans.osf"
if not OSF_PATH.exists():
    raise FileNotFoundError(f"Required fixture missing: {OSF_PATH}")

# Deterministic measurement timestamps (ns) captured from the OSF above.
EXPECTED_TIMESTAMPS = [
    6846799867874,
    6846900123055,
    6847000286697,
    6847100062339,
    6847199947465,
    6847299965077,
    6847400189932,
    6847500129960,
    6847599960666,
    6847699872269,
]

@contextmanager
def open_indexed_source():
    source = open_source(str(OSF_PATH), index=True, field_names=[])
    try:
        yield source
    finally:
        source.close()


def load_indexed_frame_sets() -> List[core.FrameSet]:
    # A plain list of FrameSets stands in for an indexed source: list slicing
    # matches the source's slicing semantics, and the snippets still see real
    # FrameSet objects (so FrameSet methods such as valid_frames() work).
    source = open_source(str(OSF_PATH), index=True)
    try:
        frames: List[core.FrameSet] = []
        for frame_tuple in source:
            converted = [
                # OSF fixture is lidar-only, so every frame is a LidarFrame
                core.LidarFrame(cast(core.LidarFrame, frame))
                for frame in frame_tuple
                if frame is not None
            ]
            if converted:
                frames.append(core.FrameSet(converted))
    finally:
        source.close()

    timestamps = [
        int(frame_set.valid_frames()[0].timestamp[0]) for frame_set in frames
    ]
    expected_prefix = EXPECTED_TIMESTAMPS[: len(timestamps)]
    assert (
        timestamps == expected_prefix
    ), "OSF fixture changed; update EXPECTED_TIMESTAMPS accordingly."

    if not frames:
        raise RuntimeError(f"No frames found in {OSF_PATH}")
    return frames


def load_frame_and_info() -> Tuple[core.LidarFrame, core.LidarInfo]:
    source = open_source(str(OSF_PATH), index=True)
    try:
        frame_ptr = source[0][0]
        if frame_ptr is None:
            raise RuntimeError(f"No frames found in {OSF_PATH}")
        # OSF fixture is lidar-only, so the frame is a LidarFrame
        frame = core.LidarFrame(cast(core.LidarFrame, frame_ptr))
        info = cast(core.LidarInfo, source.sensor_info[0])
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
    assert extract_output(capsys) == [str(EXPECTED_TIMESTAMPS[9]), str(EXPECTED_TIMESTAMPS[9])]


def test_print_last_frame_example_outputs_expected(capsys):
    source = load_indexed_frame_sets()
    snippet.print_last_frame_example(source)
    assert extract_output(capsys) == [str(EXPECTED_TIMESTAMPS[-1])]


def test_print_first_n_frames_example_iterates_expected_frames(capsys):
    source = load_indexed_frame_sets()
    snippet.print_first_n_frames_example(source)
    expected = [str(ts) for ts in EXPECTED_TIMESTAMPS[:9]]
    assert extract_output(capsys) == expected


def test_print_last_n_frames_example_iterates_expected_frames(capsys):
    source = load_indexed_frame_sets()
    snippet.print_last_n_frames_example(source)
    expected = [str(ts) for ts in EXPECTED_TIMESTAMPS[-9:-1]]
    assert extract_output(capsys) == expected


def test_print_step_sliced_frames_example_outputs_steps(capsys):
    source = load_indexed_frame_sets()
    snippet.print_step_sliced_frames_example(source)
    expected = [str(ts) for ts in EXPECTED_TIMESTAMPS[0:10:2]]
    assert extract_output(capsys) == expected


def test_slicing_as_source_example_behaves_like_slice(capsys):
    source = load_indexed_frame_sets()
    snippet.slicing_as_source_example(source)
    output = extract_output(capsys)

    expected = ["source2 length: 5", str(EXPECTED_TIMESTAMPS[5]), str(EXPECTED_TIMESTAMPS[9])]
    expected += [str(ts) for ts in EXPECTED_TIMESTAMPS[5:10]]
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
