from __future__ import annotations

from typing import Optional

# [doc-stag-lidarframe-imports]
# Other imports
# Other imports
# Other imports
# Other imports
from ouster.sdk import core
# [doc-etag-lidarframe-imports]

from ouster.sdk import open_source


def init(pcap_path: str, metadata_path: str):
    # [doc-etag-lidarframe-setup]
    source = open_source(pcap_path, meta=[metadata_path])
    sensor_info = source.sensor_info[0]
    # [doc-etag-lidarframe-setup]

    h = sensor_info.format.pixels_per_column
    w = sensor_info.format.columns_per_frame
    return source, h, w


def create_profile_frame(sensor_info) -> core.LidarFrame:
    print(f"Sensor UDP profile lidar: {sensor_info}")
    # [doc-stag-lidarframe-sensorinfo-constructor]
    # source = open_source(pcap_path, meta=[metadata_path])
    # sensor_info = source.sensor_info[0]
    profile_frame = core.LidarFrame(sensor_info)
    # [doc-etag-lidarframe-sensorinfo-constructor]
    return profile_frame


def create_reduced_frame(sensor_info) -> core.LidarFrame:
    # [doc-stag-lidarframe-reduced-slots]
    reduced_fields = [
        core.FieldType(core.ChanField.RANGE, dtype=int),
        core.FieldType(core.ChanField.NEAR_IR, dtype=int),
    ]
    reduced_frame = core.LidarFrame(sensor_info, reduced_fields)
    # [doc-etag-lidarframe-reduced-slots]
    return reduced_frame


def extract_frame_metadata(frame: core.LidarFrame):
    # [doc-stag-profile-frameid]
    frame_id = frame.frame_id
    # [doc-etag-profile-frameid]
    # [doc-stag-lidarframe-headers]
    timestamp = frame.timestamp
    status = frame.status
    measurement_id = frame.measurement_id
    # [doc-etag-lidarframe-headers]
    return {
        'frame_id': frame_id,
        'timestamp': timestamp,
        'status': status,
        'measurement_id': measurement_id
    }


def extract_field_data(frame: core.LidarFrame):
    # [doc-stag-lidarframe-fields]
    range_field = frame.field(core.ChanField.RANGE)
    reflectivity = frame.field(core.ChanField.REFLECTIVITY) # Surface reflectance values
    range2 = frame.field(core.ChanField.RANGE2) # Second return measurements (if available and enabled)
    reflectivity2 = frame.field(core.ChanField.REFLECTIVITY2)
    near_ir = frame.field(core.ChanField.NEAR_IR) # Near IR measurements
    # [doc-etag-lidarframe-fields]
    return {
        'range': range_field,
        'range2': range2,
        'reflectivity': reflectivity,
        'reflectivity2': reflectivity2
    }


def get_first_frame(source):
    """Get the first valid frame from the source."""
    for frame_set in source:
        frame = next((f for f in frame_set if f), None)
        if frame:
            return frame
    return None


def parse_args():
    import argparse

    parser = argparse.ArgumentParser(
        description="LidarFrame example using PCAP and metadata."
    )
    parser.add_argument("pcap_path", help="Path to PCAP file")
    parser.add_argument("metadata_path", help="Path to sensor metadata JSON")
    return parser.parse_args()


def main(pcap_path: str, metadata_path: str) -> None:
    source, h, w = init(pcap_path, metadata_path)
    profile_frame: Optional[core.LidarFrame] = create_profile_frame(
        source.sensor_info[0])
    reduced_fields = [
        core.FieldType(core.ChanField.RANGE, dtype=int),
        core.FieldType(core.ChanField.NEAR_IR, dtype=int),
    ]
    profile_frame = None
    reduced_frame: Optional[core.LidarFrame] = None
    for frame_set in source:
        frame = next((f for f in frame_set if f), None)
        if not frame:
            continue
        profile_frame = core.LidarFrame(frame)
        reduced_frame = core.LidarFrame(frame, reduced_fields)
        break

    if profile_frame is None:
        return

    extract_frame_metadata(profile_frame)
    extract_field_data(profile_frame)
    for name in profile_frame.fields:
        array = profile_frame.field(name)
        print(name, array.shape)


if __name__ == '__main__':
    args = parse_args()
    main(args.pcap_path, args.metadata_path)
