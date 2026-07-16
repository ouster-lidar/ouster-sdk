"""Minimal examples for iterating LidarFrame frames in Python."""
from typing import Iterable, Optional

# [doc-stag-opensource-imports]
# Other imports
from ouster.sdk import open_source
# [doc-etag-opensource-imports]
# [doc-stag-pcapframesetsource-imports]
from ouster.sdk import pcap
# [doc-etag-pcapframesetsource-imports]
from ouster.sdk import sensor
from ouster.sdk.core import LidarFrame

def stream_live(hostname: str, limit: int = 10) -> None:
    """Print a few frame ids from a live sensor."""
    # [doc-stag-single-sensorframesetsource]
    source = sensor.SensorFrameSetSource(hostname)
    # [doc-etag-single-sensorframesetsource]
    with source:
        # [doc-stag-single-sensorframesetsource-loop]
        for count, frame_set in enumerate(source):
            frame = next((f for f in frame_set if f), None)
            if frame:
                print(f"frame {count}:" +
                      f"id={frame.frame_id}")
        # [doc-etag-single-sensorframesetsource-loop]
            if count + 1 >= limit:
                break


def stream_live_via_open_source_collate(hostname: str, limit: int = 10) -> None:
    """Same as stream_live but using the high-level open_source helper."""
    # [doc-stag-single-opensource]
    source = open_source(hostname)
    # [doc-etag-single-opensource]
    with source:
        # source FrameSetSource
        # frame_set FrameSet;
        # A FrameSetSource yields FrameSet objects— each one is a small iterable containing the frames for each sensor in that time slice.
        # So for a single sensor you still get a one-element frame_set
        for count, frame_set in enumerate(source):
            # collate=True yields a list with one frame for a single sensor
            frame = frame_set[0] if frame_set and frame_set[0] else None
            if frame:
                print(f"frame {count}: id={frame.frame_id}")
            if count + 1 >= limit:
                break


def close_source(hostname: str) -> None:
    """Show explicit resource cleanup with close()."""
    # [doc-stag-single-opensource-close]
    source = open_source(hostname, sensor_idx=0)
    try:
        frame_set = next(iter(source))
        if len(frame_set) > 0 and frame_set[0]:
            print(f"first frame_id: {frame_set[0].frame_id}")
    finally:
        # Release the underlying resources: the sensor's UDP sockets for a
        # live source, or file handles for pcap/osf replay. This is the same
        # regardless of how the source was opened.
        source.close()
    # [doc-etag-single-opensource-close]


def select_single_sensor(source_url: str) -> None:
    """Two equivalent ways to restrict a source to sensor 0."""
    # [doc-stag-single-select-equivalent]
    # [doc-stag-single-opensource-nocollate]
    # Shortcut: select sensor 0 while opening. Note: sensor_idx >= 0 makes
    # open_source ignore collate entirely, so its value here has no effect.
    via_open_source = open_source(source_url, collate=False, sensor_idx=0)
    # [doc-etag-single-opensource-nocollate]

    # Equivalent: open uncollated, then derive a single-sensor view.
    via_single = open_source(source_url, collate=False).single(0)
    # [doc-etag-single-select-equivalent]
    # [doc-stag-single-opensource-nocollate-loop]
    # Both via_open_source and via_single hold live resources (sockets/file
    # handles); python context manager will close both when the with block ends.
    with via_open_source, via_single:
        # Both are Singler-wrapped, single-sensor sources, so both narrow every
        # FrameSet to one entry, accessible the same way: frame_set[0].
        via_open_source_frame_set = next(iter(via_open_source))
        via_single_frame_set = next(iter(via_single))
        if len(via_open_source_frame_set) > 0 and via_open_source_frame_set[0]:
            print(f"via sensor_idx: frame_id={via_open_source_frame_set[0].frame_id}")
        if len(via_single_frame_set) > 0 and via_single_frame_set[0]:
            print(f"via single():   frame_id={via_single_frame_set[0].frame_id}")
    # [doc-etag-single-opensource-nocollate-loop]


def replay_data(data_path: str, limit: int = 10) -> None:
    """Print frame ids while replaying from files (pcap/osf/json)."""
    # [doc-stag-pcap-replay]
    source = open_source(data_path)
    # [doc-etag-pcap-replay]
    with source:
        for count, frame_set in enumerate(source):
            frame = next((s for s in frame_set if s), None)
            if frame:
                print(f"frame {count}: id={frame.frame_id}")
                if count + 1 >= limit:
                    break

def replay_pcap_with_metadata_input(
    pcap_path: str,
    sensor_info_path: str,
    limit: int = 10,
) -> None:
    """Print frame ids while replaying from files (pcap/osf/json)."""
    # [doc-stag-pcap-replay-metadata]
    source = open_source(pcap_path,
                         meta=[sensor_info_path])
    # [doc-etag-pcap-replay-metadata]
    with source:
        for count, frame_set in enumerate(source):
            frame = next((s for s in frame_set if s), None)
            if frame:
                print(f"frame {count}: id={frame.frame_id}")
                if count + 1 >= limit:
                    break

def replay_pcap_with_pcap_frame_set_source(
    pcap_path: str,
    metadata_path: str,
    limit: int = 10,
) -> None:
    """Read PCAP using PcapFrameSetSource with explicit metadata."""
    # [doc-stag-pcapframesetsource-metadata]
    # For additional options see API documentation for PcapFrameSetSource
    source = pcap.PcapFrameSetSource(pcap_path,
                                 meta=[metadata_path])
    # [doc-etag-pcapframesetsource-metadata]
    with source:
        for count, frame_set in enumerate(source):
            frame = next((s for s in frame_set if s), None)
            if frame:
                print(f"frame {count}: id={frame.frame_id}")
                if count + 1 >= limit:
                    break


if __name__ == "__main__":
    # stream_live("os-xxxxxxxxxxxx.local")
    # select_single_sensor("os-xxxxxxxxxxxx.local")
    # replay_recording(["sample.json", "sample.pcap"])
    pass
