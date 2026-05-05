import os
import argparse
from ouster.sdk.open_source import open_source
import numpy as np


def create_stl_zone_set(data_dir, zip_path, sensor_info):
    # [doc-stag-stl-zone-set]
    from ouster.sdk.zone_monitor import Zone, ZoneSet, ZoneMode, Stl, \
        CoordinateFrame, ZoneSetOutputFilter

    # Define a zone from STL file
    stl_0 = Stl(os.path.join(data_dir, "0.stl"))
    stl_0.coordinate_frame = CoordinateFrame.BODY
    zone_0 = Zone()
    zone_0.stl = stl_0
    zone_0.point_count = 10
    zone_0.frame_count = 1
    zone_0.mode = ZoneMode.OCCUPANCY

    # Define another zone from STL file
    stl_1 = Stl(os.path.join(data_dir, "1.stl"))
    stl_1.coordinate_frame = CoordinateFrame.BODY
    zone_1 = Zone()
    zone_1.stl = stl_1
    zone_1.point_count = 20
    zone_1.frame_count = 2
    zone_1.mode = ZoneMode.VACANCY

    # Create a zone set and add the zones
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    zone_set.zones = {0: zone_0, 1: zone_1}
    zone_set.power_on_live_ids = [0, 1]

    # Print the JSON representation of the zone set
    print(zone_set.to_json(ZoneSetOutputFilter.STL))

    # Write out the zone set to a zip file
    zone_set.save(zip_path, ZoneSetOutputFilter.STL)
    # [doc-etag-stl-zone-set]


def create_zrb_zone_set(data_dir, zip_path, sensor_info):
    # [doc-stag-zrb-zone-set]
    from ouster.sdk.zone_monitor import Zone, ZoneSet, ZoneMode, Zrb, Stl, \
        CoordinateFrame, ZoneSetOutputFilter

    sensor_to_body_transform = np.eye(4)

    # Define a zone from a pair of images
    zrb = Zrb()
    zrb.near_range_mm = 10_000 * np.ones((sensor_info.h, sensor_info.w), np.uint32)
    zrb.far_range_mm = 100_000 * np.ones((sensor_info.h, sensor_info.w), np.uint32)
    zrb.serial_number = sensor_info.sn
    zrb.beam_to_lidar_transform = sensor_info.beam_to_lidar_transform
    zrb.lidar_to_sensor_transform = sensor_info.lidar_to_sensor_transform
    zrb.sensor_to_body_transform = sensor_to_body_transform

    zone_0 = Zone()
    zone_0.point_count = 100
    zone_0.frame_count = 1
    zone_0.mode = ZoneMode.OCCUPANCY
    zone_0.zrb = zrb

    # Create a second zone from an STL file
    zone_1 = Zone()
    stl_1 = Stl(os.path.join(data_dir, "1.stl"))
    zone_1.stl = stl_1
    zone_1.stl.coordinate_frame = CoordinateFrame.BODY
    zone_1.point_count = 10
    zone_1.frame_count = 1
    zone_1.mode = ZoneMode.OCCUPANCY

    # Create a zone set and add the zones
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = sensor_to_body_transform
    zone_set.power_on_live_ids = [0, 1]
    zone_set.zones = {0: zone_0, 1: zone_1}

    # render all STLs to ZRBs
    zone_set.render(sensor_info)

    # Print the JSON representation of the zone set
    print(zone_set.to_json(ZoneSetOutputFilter.ZRB))

    # Write out the zone set to a zip file
    zone_set.save(zip_path, ZoneSetOutputFilter.ZRB)
    # [doc-etag-zrb-zone-set]


def upload(zone_set, output_filter, sensor_hostname):
    # [doc-stag-upload-zone-set]
    from ouster.sdk.core import SensorHttp
    http = SensorHttp.create(args.sensor_hostname)
    print("Uploading zone monitor config...")
    http.set_zone_monitor_config_zip(zone_set_from_zip.to_zip_blob(output_filter))
    print("Applying staged config to active...")
    http.apply_zone_monitor_staged_config_to_active()
    print("Reinitializing sensor...")
    http.reinitialize()
    # [doc-etag-upload-zone-set]


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Zone monitor example")
    parser.add_argument("zone_set_type", type=str, choices=["STL", "ZRB"],
                        help="Type of zone set to create")
    parser.add_argument("data_dir", type=str,
                        help="Directory containing data files")
    parser.add_argument("zip_path", type=str,
                        help="Path to save the zone set zip file")
    parser.add_argument("sensor_hostname", type=str,
                        help="sensor hostname or IP address")
    parser.add_argument("--no-auto-udp-dest", action="store_true",
                        help="Disable auto UDP destination configuration")
    parser.add_argument("--upload", action="store_true",
                        help="Upload the zone set to the sensor")

    args = parser.parse_args()
    source = open_source(args.sensor_hostname, no_auto_udp_dest=args.no_auto_udp_dest)
    sensor_info = source.sensor_info[0]

    from ouster.sdk.zone_monitor import ZoneSetOutputFilter, ZoneSet
    output_filter = ZoneSetOutputFilter.STL if args.zone_set_type == "STL" else ZoneSetOutputFilter.ZRB
    zone_set_create_method = create_stl_zone_set if args.zone_set_type == "STL" else create_zrb_zone_set
    zone_set_create_method(args.data_dir, args.zip_path, sensor_info)

    # Load zone set from zip
    zone_set_from_zip = ZoneSet(args.zip_path)

    # how to configure sensor?
    if args.upload:
        upload(zone_set_from_zip, output_filter, args.sensor_hostname)
