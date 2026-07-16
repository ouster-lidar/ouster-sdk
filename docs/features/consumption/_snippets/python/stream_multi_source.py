"""Doc snippets for multi-sensor streams with SensorFrameSetSource/open_source."""
from typing import Iterable, Optional, Sequence
from ouster import sdk
from ouster.sdk import sensor
from ouster.sdk.core import LidarFrame


def _print_frame(count: int, frame_set: Iterable[Optional[LidarFrame]]) -> None:
    for idx, frame in enumerate(frame_set):
        if frame:
            print(f"frame: {count} frame_id={frame.frame_id}")


def stream_multi_sensor(hosts: Sequence[str], limit: int = 5) -> None:
    """Connect to multiple live sensors with SensorFrameSetSource."""
    # [doc-stag-multi-sensorframesetsource]
    # Replace with actual hostnames
    # hosts = ["os-xxxx.local", "os-yyyy.local"]
    source = sensor.SensorFrameSetSource(list(hosts))
    # [doc-etag-multi-sensorframesetsource]
    # [doc-stag-multi-sensorframesetsource-loop]
    with source:
        for count, frame_set in enumerate(source):
            for idx, _ in enumerate(frame_set):
                frame = frame_set[idx]
                if frame:
                    print(f"frame:{count} frame_id={frame.frame_id}")
        # [doc-etag-multi-sensorframesetsource-loop]
            if count + 1 >= limit:
                break


def stream_multi_open_source(hosts: Sequence[str], limit: int = 5) -> None:
    """Same as stream_multi_sensor but via open_source (collates by default)."""
    # [doc-stag-multi-opensource-sensor]
    # Replace with actual hostnames
    # hostnames = ["os-xxxx.local", "os-yyyy.local"]
    source = sdk.open_source(list(hosts), collate=True)
    with source:
        for count, frame_set in enumerate(source):
            for idx, _ in enumerate(frame_set):
                frame = frame_set[idx]
                if frame:
                    print(f"frame:{count}" f"frame_id={frame.frame_id}")

            if count + 1 >= limit:
                break
    # [doc-etag-multi-opensource-sensor]


def replay_multi_recording(paths: Sequence[str], limit: int = 5) -> None:
    """Replay multi-sensor captures (OSF/PCAP + metadata) with open_source."""
    # [doc-stag-multi-opensource-file]
    # paths = ["capture.osf", "capture2.osf"]
    source = sdk.open_source(list(paths), collate=True)
    # [doc-etag-multi-opensource-file]
    with source:
        for count, frame_set in enumerate(source):
            _print_frame(count, frame_set)
            if count + 1 >= limit:
                break


if __name__ == "__main__":
    pass
