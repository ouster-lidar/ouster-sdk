import argparse
import numpy as np
from ouster.sdk import core


def lidar_scan_example(info: core.SensorInfo) -> core.LidarScan:
    h = info.format.pixels_per_column
    w = info.format.columns_per_frame

    # ![doc-stag-python-lidarscan-constructor]
    scan = core.LidarScan(h, w, info.format.udp_profile_lidar)
    # ![doc-etag-python-lidarscan-constructor]

    # ![doc-stag-python-lidarscan-frame-id]
    frame_id = scan.frame_id  # frame_id is an int            # noqa: F841
    # ![doc-etag-python-lidarscan-frame-id]

    # ![doc-stag-python-lidarscan-headers]
    # each of these has as many entries as there are columns in the scan
    ts_0 = scan.timestamp[0]                                  # noqa: F841
    measurement_id_0 = scan.measurement_id[0]                 # noqa: F841
    status_0 = scan.status[0]                                 # noqa: F841
    # ![doc-etag-python-lidarscan-headers]

    # ![doc-stag-python-lidarscan-fields]
    # Distance measurements in millimeters
    ranges = scan.field(core.ChanField.RANGE)

    # Surface reflectance values
    reflectivity = scan.field(core.ChanField.REFLECTIVITY)    # noqa: F841

    # Near IR measurements
    near_ir = scan.field(core.ChanField.NEAR_IR)              # noqa: F841

    # Second return measurements (if available and enabled)
    ranges2 = scan.field(core.ChanField.RANGE2)               # noqa: F841
    reflectivity2 = scan.field(core.ChanField.REFLECTIVITY2)  # noqa: F841
    # ![doc-etag-python-lidarscan-fields]

    # ![doc-stag-python-scan-add-field]
    lidar_scan = core.LidarScan(128, 1024)
    lidar_scan.add_field("my-custom-field", np.uint8, ())
    lidar_scan.field("my-custom-field")[:] = 1  # set all pixels
    lidar_scan.field("my-custom-field")[10:20, 10:20] = 255  # set block of pixels
    # ![doc-etag-python-scan-add-field]

    # ![doc-stag-python-lidarscan-destagger]
    ranges = scan.field(core.ChanField.REFLECTIVITY)
    ranges_destaggered = core.destagger(info, ranges)  # noqa: F841
    # ![doc-etag-python-lidarscan-destagger]
    return lidar_scan


if __name__ == "__main__":
    # Load real sensor metadata (JSON) provided via command-line
    parser = argparse.ArgumentParser(description="LidarScan Python example")
    parser.add_argument("json_file", help="Path to sensor metadata JSON file", nargs='?')

    args, _ = parser.parse_known_args()

    if args.json_file is None:
        raise RuntimeError("Usage: python lidar_scan.py <metadata.json>")

    with open(args.json_file, "r") as f:
        info = core.SensorInfo(f.read())
        lidar_scan_example(info)
