"""Documentation snippets for FrameSetSource indexing and slicing."""
from __future__ import annotations

from ouster.sdk import core


def open_indexed_source_example(pcap_path: str = "<SAMPLE_DATA_PCAP_PATH>") -> core.FrameSetSource:
    """Doc example for enabling indexing when opening a PCAP/OSF source."""
    # [doc-stag-slicing-imports]
    from ouster import sdk
    # [doc-etag-slicing-imports]
    # [doc-stag-slicing-open-source]
    source = sdk.open_source(pcap_path,
                             index=True)
    # [doc-etag-slicing-open-source]
    return source


def print_nth_frame_example(source: core.FrameSetSource) -> None:
    """Doc example that prints the tenth frame's frame id."""
    # [doc-stag-slicing-print-nth]
    frame_set = source[9]
    for frame in frame_set:
        if frame is not None:
            print(frame.frame_id)
    # To directly access 9th frame of sensor with index 0.
    frame = source[9][0]
    assert frame is not None
    print(frame.frame_id)
    # [doc-etag-slicing-print-nth]


def print_last_frame_example(source: core.FrameSetSource) -> None:
    """Doc example that prints the last frame's frame id."""
    # [doc-stag-slicing-print-last]
    frame = source[-1][0]
    assert frame is not None
    print(frame.frame_id)
    # [doc-etag-slicing-print-last]


def print_first_n_frames_example(source: core.FrameSetSource) -> None:
    """Doc example that iterates over the first n frames."""
    # [doc-stag-slicing-first-n]
    for frame_set in source[0:9]:
        for frame in frame_set:
            if frame is not None:
                print(frame.frame_id)
    # [doc-etag-slicing-first-n]


def print_last_n_frames_example(source: core.FrameSetSource) -> None:
    """Doc example that iterates over the last nine frames."""
    # [doc-stag-slicing-last-n]
    for frame_set in source[-9:-1]:
        for frame in frame_set:
            if frame is not None:
                print(frame.frame_id)
    # [doc-etag-slicing-last-n]


def print_step_sliced_frames_example(source: core.FrameSetSource) -> None:
    """Doc example that demonstrates step slicing semantics."""
    # [doc-stag-slicing-step]
    # prints every second frame in the first ten
    for frame_set in source[0:10:2]:
        for frame in frame_set:
            if frame is not None:
                print(frame.frame_id)
    # [doc-etag-slicing-step]


def slicing_as_source_example(source: core.FrameSetSource) -> None:
    """Doc example that shows slicing returning another FrameSetSource."""
    # [doc-stag-slicing-subsource]
    source2 = source[5:10]
    # This should print 5 since source2 is scoped to the range [5, 10]
    print("source2 length:", len(source2))
    # This is equivalent to calling source[5][0]
    print(source2[0][0].frame_id)  # type: ignore[union-attr]

    # This is equivalent to calling source[9][0]
    print(source2[4][0].frame_id)  # type: ignore[union-attr]


    # Use source2 as an iterator similar to the main source, assumes only one sensor
    source_iter = iter(source2)
    for frame, in source_iter:
        assert frame is not None
        print(frame.frame_id)

    # it is possible to sub slice, meaning take the result of a previous slice operation and slice it
    # Thus, the following statement is valid
    source3 = source2[2:4]  # equivalent to source[7:9]
    print("source3 length:", len(source3))
    # [doc-etag-slicing-subsource]


def first_valid_column_example(source: core.FrameSetSource) -> None:
    """Print the first valid column index from the first frame."""
    # [doc-stag-slicing-first-valid-column]
    frame = source[0][0]
    assert frame is not None
    print(frame.get_first_valid_column())
    # [doc-etag-slicing-first-valid-column]


def last_valid_column_example(source: core.FrameSetSource) -> None:
    """Print the last valid column index from the first frame."""
    # [doc-stag-slicing-last-valid-column]
    frame = source[0][0]
    assert frame is not None
    print(frame.get_last_valid_column())
    # [doc-etag-slicing-last-valid-column]


if __name__ == "__main__":
    raise SystemExit("This module provides documentation snippets and is not executable.")
