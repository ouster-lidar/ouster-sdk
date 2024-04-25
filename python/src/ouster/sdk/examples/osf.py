import argparse
import os
import numpy as np


def osf_read_scans(osf_file: str) -> None:
    """Read Lidar Scans from an OSF file.

    Shows scans only for a single sensor, whatever happened to be the first stored in an OSF file.
    """
    import ouster.sdk.osf as osf
    # [doc-stag-osf-read-scans]
    scans = osf.Scans(osf_file)
    for scan in scans:
        print(f'scan = {scan}, WxH={scan.w}x{scan.h}')

    # or with timestamps
    for ts, scan in scans.withTs():
        print(f'ts = {ts}, scan = {scan}, WxH={scan.w}x{scan.h}')
    # [doc-etag-osf-read-scans]


def osf_get_sensors_info(osf_file: str) -> None:
    """Read Lidar Sensors info from an OSF file.

    Shows metadata for all sensors found in an OSF file.
    """
    import ouster.sdk.osf as osf
    # [doc-stag-osf-get-sensors-info]
    reader = osf.Reader(osf_file)
    # Get all stored sensors information
    sensors = reader.meta_store.find(osf.LidarSensor)
    for sensor_id, sensor_meta in sensors.items():
        info = sensor_meta.info
        print(f"sensor[{sensor_id}] = ", info)
    # [doc-etag-osf-get-sensors-info]


def osf_read_all_messages(osf_file: str) -> None:
    """Read all message from an OSF file."""
    import ouster.sdk.osf as osf
    # [doc-stag-osf-read-all-messages]
    reader = osf.Reader(osf_file)
    # Reading all messages
    for msg in reader.messages():
        print(f'ts = {msg.ts}, stream_im = {msg.id}')
        if msg.of(osf.LidarScanStream):
            scan = msg.decode()
            print(f'  got lidar scan = {scan.h}x{scan.w}')
    # [doc-etag-osf-read-all-messages]


def osf_check_layout(osf_file: str) -> None:
    """Checks chunks layout of an OSF file.

    Open file and checks for the presence of osf.StreamingInfo metadata entry, which signals the
    STREAMING layout OSF. Also reads StreamingInfo and prints message counts and avg size of per
    stream.

    NOTE: All OSFs produced from June 15, 2022 have the STREAMING layout which the
          default and only layout available.
    """
    import ouster.sdk.osf as osf
    # [doc-stag-osf-check-layout]
    reader = osf.Reader(osf_file)
    # finds the first StreamingInfo metadata entry if any present
    streaming_info = reader.meta_store.get(osf.StreamingInfo)
    if streaming_info:
        print("Stats available (STREAMING layout):")
        for stream_id, stream_stat in streaming_info.stream_stats:
            msg_cnt = stream_stat.message_count
            msg_avg_size = stream_stat.message_avg_size
            print(f"  stream[{stream_id}]: msg_count = {msg_cnt},",
                  f"msg_avg_size = {msg_avg_size}")
    else:
        print("No stats available (STANDARD layout)")
    # [doc-etag-osf-check-layout]


def osf_get_lidar_streams(osf_file: str) -> None:
    """Reads info about available Lidar Scan streams in an OSF file.

    Find all LidarScanStream metadata entries and prints sensor_meta_id with encoded field types of
    a LidarScan in a messages.
    """
    import ouster.sdk.osf as osf
    # [doc-stag-osf-get-lidar-streams]
    reader = osf.Reader(osf_file)
    lidar_streams = reader.meta_store.find(osf.LidarScanStream)
    for stream_id, stream_meta in lidar_streams.items():
        sensor_id = stream_meta.sensor_meta_id
        field_types = stream_meta.field_types
        print(f"LidarScanStream[{stream_id}]:")
        print(f"  sensor_id = {sensor_id}")
        print(f"  field_types = {field_types}")
    # [doc-etag-osf-get-lidar-streams]


def osf_slice_scans(osf_file: str) -> None:
    """Copy scans from input OSF file with reduction using the Writer API.

    Slicing is done via saving only RANGE, SIGNAL and REFLECTIVITY fields into an output OSF files.
    """
    from ouster.sdk import client
    import ouster.sdk.osf as osf
    # [doc-stag-osf-slice-scans]
    # Scans reader from input OSF
    scans = osf.Scans(osf_file)

    # New field types should be a subset of fields in encoded LidarScan so we just assume that
    # RANGE, SIGNAL and REFLECTIVITY fields will be present in the input OSF file.
    new_field_types = dict({
        client.ChanField.RANGE: np.dtype('uint32'),
        client.ChanField.SIGNAL: np.dtype('uint16'),
        client.ChanField.REFLECTIVITY: np.dtype('uint16')
    })

    output_file_base = os.path.splitext(os.path.basename(osf_file))[0]
    output_file = output_file_base + '_sliced.osf'

    # Create Writer with a subset of fields to save (i.e. slicing will happen
    # automatically on write)
    writer = osf.Writer(output_file, scans.metadata, new_field_types)

    # Read scans and write back
    for ts, scan in scans.withTs():
        print(f"writing sliced scan with ts = {ts}")
        writer.save(0, scan, ts)

    writer.close()
    # [doc-etag-osf-slice-scans]


def osf_split_scans(osf_file: str) -> None:
    """Splits scans from input OSF into N=2 files.

    Spliting is done by timestamp.
    """
    import ouster.sdk.osf as osf
    # [doc-stag-osf-split-scans]
    reader = osf.Reader(osf_file)
    start_ts = reader.start_ts
    end_ts = reader.end_ts
    n_splits = 2
    split_dur = int((end_ts - start_ts) / n_splits)

    # Scans reader from input OSF
    scans = osf.Scans(osf_file)

    output_file_base = os.path.splitext(os.path.basename(osf_file))[0]

    # Create N writers and create N output Lidar Streams to write too
    writers = []
    for i in range(n_splits):
        writers.append(osf.Writer(f"{output_file_base}_s{i:02d}.osf", scans.metadata))

    # Read scans and write to a corresponding output stream
    for ts, scan in scans.withTs():
        split_idx = int((ts - start_ts) / split_dur)
        print(f"writing scan to split {split_idx:02d} file")
        writers[split_idx].save(0, scan)

    # No need to call close, underlying writers will close automatically on destroy
    # [doc-etag-osf-split-scans]


def main():
    """OSF examples runner."""
    examples = {
        "read-scans": osf_read_scans,
        "read-messages": osf_read_all_messages,
        "split-scans": osf_split_scans,
        "slice-scans": osf_slice_scans,
        "get-lidar-streams": osf_get_lidar_streams,
        "get-sensors-info": osf_get_sensors_info,
        "check-layout": osf_check_layout
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
