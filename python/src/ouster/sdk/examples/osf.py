import argparse
import os


def osf_read_scans(osf_file: str) -> None:
    """Read Lidar Scans from an OSF file.

    Shows scans for each sensor in time order stored in the OSF File.
    """
    import ouster.sdk.osf as osf
    # [doc-stag-osf-read-scans]
    scans = osf.OsfScanSource(osf_file)
    for scan, in scans:
        assert scan is not None
        print(f'scan = {scan}, WxH={scan.w}x{scan.h}')
    # [doc-etag-osf-read-scans]


def osf_get_sensors_info(osf_file: str) -> None:
    """Read Lidar Sensors info from an OSF file.

    Shows metadata for all sensors found in an OSF file.
    """
    import ouster.sdk.osf as osf
    # [doc-stag-osf-get-sensors-info]
    scans = osf.OsfScanSource(osf_file)
    for sensor_id, info in enumerate(scans.sensor_info):
        print(f"sensor[{sensor_id}] = ", info)
    # [doc-etag-osf-get-sensors-info]


def osf_slice_scans(osf_file: str) -> None:
    """Copy scans from input OSF file with reduction using the Writer API.

    Slicing is done via saving only RANGE, SIGNAL and REFLECTIVITY fields into an output OSF files.
    """
    from ouster.sdk import core
    import ouster.sdk.osf as osf
    # [doc-stag-osf-slice-scans]
    # Scans reader from input OSF
    scans = osf.OsfScanSource(osf_file)

    # New field types should be a subset of fields in encoded LidarScan so we just assume that
    # RANGE, SIGNAL and REFLECTIVITY fields will be present in the input OSF file.
    fields_to_write = [core.ChanField.RANGE, core.ChanField.SIGNAL, core.ChanField.REFLECTIVITY]

    output_file_base = os.path.splitext(os.path.basename(osf_file))[0]
    output_file = output_file_base + '_sliced.osf'

    # Create Writer with a subset of fields to save (i.e. slicing will happen
    # automatically on write)
    writer = osf.Writer(output_file, scans.sensor_info, fields_to_write)

    # Read scans and write back
    for scanl in scans:
        for idx, scan in enumerate(scanl):
            if scan is None:
                continue
            print(f"writing sliced scan with ts = {scan.get_first_valid_packet_timestamp()}")
            writer.save(idx, scan)

    writer.close()
    # [doc-etag-osf-slice-scans]


def main():
    """OSF examples runner."""
    examples = {
        "read-scans": osf_read_scans,
        "slice-scans": osf_slice_scans,
        "get-sensors-info": osf_get_sensors_info,
    }

    description = "Ouster Python SDK OSF examples. The EXAMPLE must be one of:\n  " + str.join(
        '\n  ', examples.keys())

    parser = argparse.ArgumentParser(
        description=description, formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('osf_path', metavar='OSF', help='path to osf file')
    parser.add_argument('example',
                        metavar='EXAMPLE',
                        choices=examples.keys(),
                        help='name of the example to run')
    parser.add_argument('--scan-num',
                        type=int,
                        default=1,
                        help='index of scan to use')
    args = parser.parse_args()

    try:
        example = examples[args.example]
    except KeyError:
        print(f"No such example: {args.example}")
        print(description)
        exit(1)

    print(f'example: {args.example}')

    example(osf_file=args.osf_path)  # type: ignore


if __name__ == "__main__":
    main()
