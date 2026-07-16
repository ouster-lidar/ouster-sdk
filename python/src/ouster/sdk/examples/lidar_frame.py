import argparse
import numpy as np
from ouster.sdk import core


def lidar_frame_example(info: core.SensorInfo) -> core.LidarFrame:
    # ![doc-stag-python-lidarframe-constructor]
    frame = core.LidarFrame(info)
    # ![doc-etag-python-lidarframe-constructor]

    # ![doc-stag-python-lidarframe-frame-id]
    frame_id = frame.frame_id  # frame_id is an int            # noqa: F841
    # ![doc-etag-python-lidarframe-frame-id]

    # ![doc-stag-python-lidarframe-headers]
    # each of these has as many entries as there are columns in the frame
    ts_0 = frame.timestamp[0]                                  # noqa: F841
    measurement_id_0 = frame.measurement_id[0]                 # noqa: F841
    status_0 = frame.status[0]                                 # noqa: F841
    # ![doc-etag-python-lidarframe-headers]

    # ![doc-stag-python-lidarframe-fields]
    # Distance measurements in millimeters
    ranges = frame.field(core.ChanField.RANGE)

    # Surface reflectance values
    reflectivity = frame.field(core.ChanField.REFLECTIVITY)    # noqa: F841

    # Near IR measurements
    near_ir = frame.field(core.ChanField.NEAR_IR)              # noqa: F841

    # Second return measurements (if available and enabled)
    ranges2 = frame.field(core.ChanField.RANGE2)               # noqa: F841
    reflectivity2 = frame.field(core.ChanField.REFLECTIVITY2)  # noqa: F841
    # ![doc-etag-python-lidarframe-fields]

    # ![doc-stag-python-frame-add-field]
    lidar_frame = core.LidarFrame(info, [])
    lidar_frame.add_field("my-custom-field", np.uint8, ())
    lidar_frame.field("my-custom-field")[:] = 1  # set all pixels
    lidar_frame.field("my-custom-field")[10:20, 10:20] = 255  # set block of pixels
    # ![doc-etag-python-frame-add-field]

    # ![doc-stag-python-lidarframe-destagger]
    ranges = frame.field(core.ChanField.REFLECTIVITY)
    ranges_destaggered = core.destagger(info, ranges)  # noqa: F841
    # ![doc-etag-python-lidarframe-destagger]
    return lidar_frame


if __name__ == "__main__":
    # Load real sensor metadata (JSON) provided via command-line
    parser = argparse.ArgumentParser(description="LidarFrame Python example")
    parser.add_argument("json_file", help="Path to sensor metadata JSON file", nargs='?')

    args, _ = parser.parse_known_args()

    if args.json_file is None:
        raise RuntimeError("Usage: python lidar_frame.py <metadata.json>")

    with open(args.json_file, "r") as f:
        info = core.SensorInfo(f.read())
        lidar_frame_example(info)
