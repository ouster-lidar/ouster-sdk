"""Executable examples for using the pcap APIs.

This module has a rudimentary command line interface. For usage, run::

    $ python -m ouster.sdk.examples.pcap -h
"""

import os
import argparse
import numpy as np
from contextlib import closing
from more_itertools import nth

import ouster.pcap as pcap
import ouster.client as client


def read_metadata(metadata_path: str) -> client.SensorInfo:
    """Read metadata json file as :class:`.client.SensorInfo` object

    Args:
        metadata_path: path to json file with sensor info

    Returns:
        :class:`.SensorInfo`: object initialized from the give json file
    """
    with open(metadata_path, 'r') as f:
        return client.SensorInfo(f.read())


def ae(field: np.ndarray, percentile: float = 0.05):
    """Shift/Normalize for better color mapping (non reversible, only viz use
    cases)

    WARNING:
        It's a utility function used ONLY for the purpose of 2D images
        visualization, resulting values are not fully reversible (because
        of final clipping step that discards values outside of [0,1] range),
        so you can't use this function in serious processing of Lidar Data
        because of information loss.

    Args:
        field: field values. Usually returned from channel field
               :class:`.ChanField` of a scan (can be staggered or destaggered)
        percentile: param to control the amount of high/low value outliers
                    removed from the *field*.

    Returns:
        :class:`.ndarray`: Output array of the same shape as *field* but values
        between [0..1] that produce better color ranges (tones and highlights)
        when visualizing.
    """
    min_val = np.percentile(field, 100 * percentile)
    max_val = np.percentile(field, 100 * (1 - percentile))
    field_res = (field.astype(np.float64) - min_val) / (max_val - min_val)
    return field_res.clip(0, 1.0)


def pcap_display_xyz_points(pcap_path: str,
                            metadata_path: str,
                            num: int = 0) -> None:
    """Display range from a specified scan number (*num*) as 3D points from
    pcap file located at *pcap_path*

    Args:
        pcap_path: path to the pcap file
        metadata_path: path to the .json with metadata (aka :class:`.SensorInfo`)
        num: scan number in a given pcap file (satrs from *0*)
    """
    import matplotlib.pyplot as plt  # type: ignore

    # [doc-stag-pcap-plot-xyz-points]
    metadata = read_metadata(metadata_path)
    source = pcap.Pcap(pcap_path, metadata)

    # get single scan
    scans = client.Scans(source)
    scan = nth(scans, num)
    if not scan:
        print(f'ERROR: Scan # {num} in not present in pcap file: {pcap_path}')
        return

    # set up figure
    plt.figure()
    ax = plt.axes(projection='3d')
    r = 6
    ax.set_xlim3d([-r, r])
    ax.set_ylim3d([-r, r])
    ax.set_zlim3d([-r, r])

    plt.title("3D Points XYZ for scan")

    # transform data to 3d points and graph
    xyzlut = client.XYZLut(metadata)
    xyz = xyzlut(scan)

    key = scan.field(client.ChanField.SIGNAL)

    [x, y, z] = [c.flatten() for c in np.dsplit(xyz, 3)]
    ax.scatter(x, y, z, c=ae(key.flatten()), s=0.2)
    plt.show()

    # [doc-etag-pcap-plot-xyz-points]


def pcap_2d_viewer(
        pcap_path: str,
        metadata_path: str,
        num: int = 0,  # not used in this example
        rate: float = 0.0) -> None:
    """Simple sensor field visualization pipeline as 2D images from pcap file
    (*pcap_path*)

    Args:
        pcap_path: path to the pcap file
        metadata_path: path to the .json with metadata (aka :class:`.SensorInfo`)
        rate: read speed of packets from the pcap file (**1.0** - corresponds to
              real-time by packets timestamp, **0.0** - as fast as it reads from
              file without any delay)

    """
    import cv2  # type: ignore

    # [doc-stag-pcap-display-live]
    metadata = read_metadata(metadata_path)

    source = pcap.Pcap(pcap_path, metadata, rate=rate)

    with closing(source) as source:
        scans = iter(client.Scans(source))

        print("press ESC from visualization to exit")

        channels = [
            client.ChanField.RANGE, client.ChanField.SIGNAL,
            client.ChanField.NEAR_IR, client.ChanField.REFLECTIVITY
        ]

        paused = False
        destagger = True
        num = 0
        scan = next(scans, None)
        while scan:
            print("frame id: {}, num = {}".format(scan.frame_id, num))

            fields_values = [scan.field(ch) for ch in channels]

            if destagger:
                fields_values = [
                    client.destagger(metadata, field_val)
                    for field_val in fields_values
                ]

            fields_images = [ae(field_val) for field_val in fields_values]

            combined_images = np.vstack(
                [np.pad(img, 2, constant_values=1.0) for img in fields_images])

            cv2.imshow("4 channels: ", combined_images)

            key = cv2.waitKey(1) & 0xFF
            # 100 is d
            if key == 100:
                destagger = not destagger
            # 32 is SPACE
            if key == 32:
                paused = not paused
            # 27 is ESC
            elif key == 27:
                break

            if not paused:
                scan = next(scans, None)
                num += 1

        cv2.destroyAllWindows()
    # [doc-etag-pcap-display-live]


def pcap_read_packets(
        pcap_path: str,
        metadata_path: str,
        num: int = 0  # not used in this examples
) -> None:
    """Basic read packets example from pcap file (*pcap_path*)

    Args:
        pcap_path: path to the pcap file
        metadata_path: path to the .json with metadata (aka :class:`.SensorInfo`)
    """

    metadata = read_metadata(metadata_path)
    source = pcap.Pcap(pcap_path, metadata)

    # [doc-stag-pcap-read-packets]
    for packet in source:
        if isinstance(packet, client.LidarPacket):
            # Now we can process the LidarPacket. In this case, we access
            # the encoder_counts, timestamps, and ranges
            encoder_counts = packet.header(client.ColHeader.ENCODER_COUNT)
            timestamps = packet.header(client.ColHeader.TIMESTAMP)
            ranges = packet.field(client.ChanField.RANGE)
            print(f'  encoder counts = {encoder_counts.shape}')
            print(f'  timestamps = {timestamps.shape}')
            print(f'  ranges = {ranges.shape}')

        if isinstance(packet, client.ImuPacket):
            # and access ImuPacket content
            print(f'  acceleration = {packet.accel}')
            print(f'  angular_velocity = {packet.angular_vel}')
    # [doc-etag-pcap-read-packets]


def pcap_show_one_scan(pcap_path: str,
                       metadata_path: str,
                       num: int = 0,
                       destagger: bool = True) -> None:
    """Show all 4 channels of one scan (*num*) form pcap file (*pcap_path*)

    Args:
        pcap_path: path to the pcap file
        metadata_path: path to the .json with metadata (aka :class:`.SensorInfo`)
        num: scan number in a given pcap file (satrs from *0*)
    """
    import matplotlib.pyplot as plt  # type: ignore

    # [doc-stag-pcap-show-one]
    metadata = read_metadata(metadata_path)

    def prepare_field_image(scan, key, metadata, destagger=True):
        f = ae(scan.field(key))
        if destagger:
            return client.destagger(metadata, f)
        return f

    show_fields = [('range', client.ChanField.RANGE),
                   ('signal', client.ChanField.SIGNAL),
                   ('near_ir', client.ChanField.NEAR_IR),
                   ('reflectivity', client.ChanField.REFLECTIVITY)]

    with closing(pcap.Pcap(pcap_path, metadata)) as source:
        scan = nth(client.Scans(source), num)
        if not scan:
            return

        fields_images = [(sf[0],
                          prepare_field_image(scan, sf[1], source.metadata))
                         for sf in show_fields]

        fig = plt.figure(constrained_layout=True)

        axs = fig.subplots(len(fields_images), 1, sharey=True)

        for ax, field in zip(axs, fields_images):
            ax.set_title(field[0], fontdict={'fontsize': 10})
            ax.imshow(field[1], cmap='gray', resample=False)
            ax.set_yticklabels([])
            ax.set_yticks([])
            ax.set_xticks([0, scan.w])
        plt.show()
    # [doc-etag-pcap-show-one]


def pcap_to_csv(pcap_path: str,
                metadata_path: str,
                num: int = 0,
                csv_dir: str = ".",
                csv_prefix: str = "pcap_out",
                csv_ext: str = "csv") -> None:
    """Write scans from pcap file (*pcap_path*) to plain csv files (one per
    lidar scan).

    If the *csv_ext* ends in ``.gz``, the file is automatically saved in
    compressed gzip format. :func:`.numpy.loadtxt` can be used to read gzipped
    files transparently back to :class:`.numpy.ndarray`.

    Number of saved lines per csv file is always [H x W], which corresponds
    to a full 2D image representation of a lidar scan.

    Each line in a csv file is:

        RANGE (mm), SIGNAL, NEAR_IR, REFLECTIVITY, X (m), Y (m), Z (m)

    Args:
        pcap_path: path to the pcap file
        metadata_path: path to the .json with metadata (aka :class:`.SensorInfo`)
        num: number of scans to save from pcap to csv files
        csv_dir: path to the directory where csv files will be saved
        csv_prefix: the filename prefix that will be appended with frame number
                    and *csv_ext*
        csv_ext: file extension to use. If it ends with ``.gz`` the output is
                 gzip compressed

    """
    from itertools import islice

    # ensure that base csv_dir exists
    if not os.path.exists(csv_dir):
        os.makedirs(csv_dir)

    metadata = read_metadata(metadata_path)
    source = pcap.Pcap(pcap_path, metadata)

    # [doc-stag-pcap-to-csv]
    field_names = 'RANGE (mm), SIGNAL, NEAR_IR, REFLECTIVITY, X (m), Y (m), Z (m)'
    field_fmts = ['%d', '%d', '%d', '%d', '%.8f', '%.8f', '%.8f']

    channels = [
        client.ChanField.RANGE, client.ChanField.SIGNAL,
        client.ChanField.NEAR_IR, client.ChanField.REFLECTIVITY
    ]

    with closing(pcap.Pcap(pcap_path, metadata)) as source:

        # precompute xyzlut to save computation in a loop
        xyzlut = client.XYZLut(metadata)

        # create an iterator of LidarScans from pcap and bound it if num is specified
        scans = iter(client.Scans(source))
        if num:
            scans = islice(scans, num)

        for idx, scan in enumerate(scans):

            fields_values = [scan.field(ch) for ch in channels]
            xyz = xyzlut(scan)

            # get lidar data as one frame of [H x W x 7], "fat" 2D image
            frame = np.dstack((*fields_values, xyz))
            frame = client.destagger(metadata, frame)

            csv_path = os.path.join(csv_dir,
                                    f'{csv_prefix}_{idx:06d}.{csv_ext}')

            header = '\n'.join([
                f'pcap file: {pcap_path}', f'frame num: {idx}',
                f'metadata file: {metadata_path}', field_names
            ])

            print(f'write frame #{idx}, to file: {csv_path}')

            np.savetxt(csv_path,
                       np.reshape(frame, (-1, frame.shape[2])),
                       fmt=field_fmts,
                       delimiter=',',
                       header=header)

    # [doc-etag-pcap-to-csv]


def main():

    examples = {
        "plot-xyz-points": pcap_display_xyz_points,
        "2d-viewer": pcap_2d_viewer,
        "read-packets": pcap_read_packets,
        "plot-one-scan": pcap_show_one_scan,
        "pcap-to-csv": pcap_to_csv
    }

    description = "Ouster Python SDK Pcap examples. The EXAMPLE must be one of:\n  " + str.join(
        '\n  ', examples.keys())

    parser = argparse.ArgumentParser(
        description=description, formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('pcap_path',
                        type=str,
                        help='pcap file with Ouster data packets')

    parser.add_argument(
        'metadata_path',
        type=str,
        help='sensor info .json file (usually stored together with recordings)'
    )

    parser.add_argument('example',
                        metavar='EXAMPLE',
                        type=str,
                        choices=examples.keys(),
                        help='Name of the example to run')

    parser.add_argument('--scan-num',
                        dest='scan_num',
                        type=int,
                        default=0,
                        help='LidarScan # to use',
                        required=False)

    args = parser.parse_args()

    print(f'Check basic pcap file info: {args.pcap_path}')
    pcap_info = pcap.info(args.pcap_path)
    print('pcap_info = ', pcap_info)

    try:
        example = examples[args.example]
    except KeyError:
        print(f'No such example: {args.example}')
        print(description)
        exit(1)

    if not args.metadata_path:
        print("WARNING: Can't do more without sensor info .json "
              "file, please provide one with --info-json param")
        return 1

    print(f'example: {args.example}')
    example(args.pcap_path, args.metadata_path, args.scan_num)  # type: ignore


if __name__ == "__main__":
    main()
