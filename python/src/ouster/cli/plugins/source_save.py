from ouster.cli.core.cli_args import CliArgs
import atexit
import click
import os
import time
from datetime import datetime
from pathlib import Path
import numpy as np
from typing import (Tuple, List, Iterator, Union)
from ouster.cli.core import SourceArgsException  # type: ignore[attr-defined]
from ouster.sdk.client import (first_valid_packet_ts,
                               first_valid_column_ts,
                               UDPProfileLidar, LidarScan, ChanField, XYZLut,
                               ScanSource, destagger, SensorInfo,
                               LidarPacket, ImuPacket, ScanSourceAdapter)
from ouster.sdk import osf
from ouster.sdk.io_type import (io_type_from_extension,
                                OusterIoType)
from ouster.sdk.pcap import BagRecordingPacketSource, RecordingPacketSource, PcapScanSource
from ouster.sdk.sensor import SensorScanSource
from ouster.sdk.util import scan_to_packets  # type: ignore
from ouster.sdk.pcap.pcap import MTU_SIZE
import ouster.sdk.pcap._pcap as _pcap
from .source_util import (SourceCommandContext,
                          SourceCommandType,
                          source_multicommand,
                          _join_with_conjunction,
                          import_rosbag_modules)
from contextlib import closing

_file_exists_error = lambda filename: (f"Error: File '{filename}' already exists. Add --overwrite "
                                       "flag to overwrite and continue anyways.")


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('--chunk-size', default=0, help="Split output by size (MB)")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.option('-r', '--raw', is_flag=True, default=False, help="Save in raw mode, "
              "where LidarPackets and ImuPackets from compatible sources are saved directly. "
              "This mode does not preserve LidarScan transformations performed by other commands "
              "in a multi-command chain. This mode preserves LEGACY ImuPackets.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_save_pcap(ctx: SourceCommandContext, prefix: str, dir: str, filename: str,
                     chunk_size: int, raw: bool, overwrite: bool, **kwargs) -> None:
    """Save source as a PCAP"""
    scan_source = ctx.scan_source
    scans = ctx.scan_iter
    info = scan_source.metadata  # type: ignore

    # Automatic file naming
    filename = determine_filename(filename=filename, info=info, extension=".pcap", prefix=prefix, dir=dir)

    create_directories_if_missing(filename)

    filename = filename[0:-5]  # remove extension

    if os.path.isfile(f'{filename}.json') and not overwrite:
        click.echo(_file_exists_error(f'{filename}.json'))
        exit(1)

    # Save metadata as json
    with open(f"{filename}.json", 'w') as f:
        f.write(info.updated_metadata_string())

    if raw:
        scan_source = None
        if isinstance(ctx.scan_source, ScanSourceAdapter):
            if isinstance(ctx.scan_source._scan_source, SensorScanSource):
                scan_source = ctx.scan_source._scan_source._scans  # type: ignore
            elif isinstance(ctx.scan_source._scan_source, PcapScanSource):
                scan_source = ctx.scan_source._scan_source  # type: ignore

        if scan_source is None:
            # [kk] TODO: Only single-source via ScanSourceAdapter is currently supported.
            # Revisit when implmenting multi source in CLI
            raise click.exceptions.BadParameter("Saving in -r/--raw mode is not supported with "
                                                "the current source type.")

        if len(ctx.invoked_command_names) != 1:
            click.echo("Warning: Saving pcap in -r/--raw mode will drop any LidarScan "
                       "transformations perfomed by other commands in this multi-command chain: "
                       f"{', '.join([c for c in ctx.invoked_command_names if c != 'save'])}.")

        # replace ScanSource's packetsource with RecordingPacketSource
        scan_source._source = RecordingPacketSource(
            scan_source._source, n_frames=None,
            prefix_path=filename, chunk_size=chunk_size, overwrite=overwrite,
            lidar_port=info.udp_port_lidar, imu_port=info.udp_port_imu
        )
    else:
        click.echo("Warning: Saving pcap without -r/--raw will not save LEGACY IMU packets.")

        if os.path.isfile(f'{filename}.pcap') and not overwrite:
            click.echo(_file_exists_error(f'{filename}.pcap'))
            exit(1)

        # Initialize pcap writer
        pcap_record_handle = _pcap.record_initialize(f"{filename}.pcap", MTU_SIZE, False)

        def save_iter():
            try:
                for scan in scans:
                    # [kk] TODO: implement chunk-size
                    packets = scan_to_packets(scan, info)
                    for packet in packets:
                        ts = packet.capture_timestamp or time.time()
                        _pcap.record_packet(pcap_record_handle, "127.0.0.1",
                                            "127.0.0.1", info.udp_port_lidar,
                                            info.udp_port_lidar, packet._data, ts)
                    yield scan
            except (KeyboardInterrupt, StopIteration):
                pass
            finally:
                # Finish pcap_recording when this generator is garbage collected
                _pcap.record_uninitialize(pcap_record_handle)
        click.echo(f"Saving PCAP file at {filename}.pcap")
        ctx.scan_iter = save_iter()

    click.echo(f"Saving metadata json at {filename}.json")


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('-c', '--continue-anyways', is_flag=True, default=False, help="Continue saving "
              "scans after an error is encountered, dropping bad data if necessary.")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.option("--ts", default='packet', help="Timestamp to use for indexing.", type=click.Choice(['packet', 'lidar']))
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_save_osf(ctx: SourceCommandContext, prefix: str, dir: str, filename: str,
                    overwrite: bool, ts: str, continue_anyways: bool, **kwargs) -> None:
    """Save source as an OSF"""
    scans = ctx.scan_iter
    info = ctx.scan_source.metadata  # type: ignore

    # Automatic file naming
    filename = determine_filename(filename=filename, info=info, extension=".osf", prefix=prefix, dir=dir)

    create_directories_if_missing(filename)

    click.echo(f"Saving OSF file at {filename}")

    if os.path.isfile(filename) and not overwrite:
        click.echo(_file_exists_error(filename))
        exit(1)

    # Initialize osf writer
    osf_writer = osf.Writer(filename, info)

    # TODO: extrinsics still need to be plugged in here -- Tim T.
    wrote_scans = False
    dropped_scans = 0
    ts_method = first_valid_packet_ts if ts == "packet" else first_valid_column_ts
    last_ts = 0

    # returns false if we should stop recording
    def write_osf(scan: LidarScan):
        nonlocal wrote_scans, last_ts, dropped_scans
        # Set OSF timestamp to the timestamp of the first valid column
        scan_ts = ts_method(scan)
        if scan_ts:
            if scan_ts < last_ts:
                if continue_anyways:
                    dropped_scans = dropped_scans + 1
                    return True
                else:
                    print("WARNING: Stopped saving because scan timestamps jumped backwards which is "
                          "not supported by OSF. Try with `-c` to drop these scans and continue "
                          "anyways.")
                    osf_writer.close()
                    return False
            wrote_scans = True
            osf_writer.save(0, scan, scan_ts)
            last_ts = scan_ts
        else:
            # by default fail out
            if not continue_anyways:
                osf_writer.close()
                os.remove(filename)
                print("ERROR: Cannot save scans because they are missing packet timestamps."
                      " Try with `--ts lidar` instead or `-c` to continue anyways.")
                raise ValueError("Bad timestamps")
            dropped_scans = dropped_scans + 1
        return True

    def save_iter():
        try:
            with closing(osf_writer):
                stop = False
                for scan in scans:
                    # Drop invalid lidarscans
                    if not stop and np.any(scan.status):
                        if not write_osf(scan):
                            stop = True
                    yield scan
        except (KeyboardInterrupt):
            pass
        except (ValueError):
            ctx.terminate_evt.set()

    ctx.scan_iter = save_iter()

    def exit_print():
        if dropped_scans > 0:
            if ts == "lidar":
                click.echo(f"WARNING: Dropped {dropped_scans} scans because missing or decreasing timestamps.")
            else:
                click.echo(f"WARNING: Dropped {dropped_scans} scans because missing or decreasing "
                           "packet timestamps. Try with `--ts lidar` instead.")
        if not wrote_scans:
            click.echo("WARNING: No scans saved.")
    atexit.register(exit_print)


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_save_csv(ctx: SourceCommandContext, prefix: str,
                    dir: str, filename: str, overwrite: bool, **kwargs) -> None:
    """Save source as one CSV file per LidarScan."""
    ctx.scan_iter = source_to_csv_iter(ctx.scan_iter, ctx.scan_source.metadata,  # type: ignore
                                       prefix=prefix, dir=dir, filename=filename,
                                       overwrite=overwrite)


def source_to_csv_iter(scan_iter: Iterator[LidarScan], info: SensorInfo,
                       prefix: str = "", dir: str = "", overwrite: bool = True,
                       filename: str = "") -> Iterator[LidarScan]:
    """Create a CSV saving iterator from a LidarScan iterator

    The number of saved lines per csv file is always H x W, which corresponds to
    a full 2D image representation of a lidar scan.

    Each line in a csv file is (for DUAL profile):

        TIMESTAMP (ns), RANGE (mm), RANGE2 (mm), SIGNAL (photons),
            SIGNAL2 (photons), REFLECTIVITY (%), REFLECTIVITY2 (%),
            NEAR_IR (photons), X (m), Y (m), Z (m), X2 (m), Y2 (m), Z2(m),
            MEASUREMENT_ID, ROW, COLUMN
    """

    dual = False
    if info.format.udp_profile_lidar in [UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL]:
        dual = True
        print("Note: You've selected to convert a dual returns pcap to CSV. Each row "
              "will represent a single pixel, so that both returns for that pixel will "
              "be on a single row. As this is an example we provide for getting "
              "started, we realize that you may have conversion needs which are not met "
              "by this function. You can find the source code on the Python SDK "
              "documentation website to modify it for your own needs.")

    # Build filename
    filename = determine_filename(filename=filename, info=info, extension=".csv", prefix=prefix, dir=dir)

    create_directories_if_missing(filename)

    filename = filename[0:-4]  # remove extension

    click.echo(f"Saving CSV file at {filename}.csv")

    # Construct csv header and data format
    def get_fields_info(scan: LidarScan) -> Tuple[str, List[str]]:
        field_names = 'TIMESTAMP (ns), ROW, DESTAGGERED IMAGE COLUMN, MEASUREMENT_ID'
        field_fmts = ['%d'] * 4
        for chan_field in scan.fields:
            field_names += f', {chan_field}'
            if chan_field in [ChanField.RANGE, ChanField.RANGE2]:
                field_names += ' (mm)'
            if chan_field in [ChanField.REFLECTIVITY, ChanField.REFLECTIVITY2]:
                field_names += ' (%)'
            if chan_field in [ChanField.SIGNAL, ChanField.SIGNAL2,
                    ChanField.NEAR_IR]:
                field_names += ' (photons)'
            field_fmts.append('%d')
        field_names += ', X1 (m), Y1 (m), Z1 (m)'
        field_fmts.extend(3 * ['%.4f'])
        if dual:
            field_names += ', X2 (m), Y2 (m), Z2 (m)'
            field_fmts.extend(3 * ['%.4f'])
        return field_names, field_fmts

    field_names: str = ''
    field_fmts: List[str] = []

    # [doc-stag-pcap-to-csv]
    # {recompute xyzlut to save computation in a loop
    xyzlut = XYZLut(info)

    row_layer = np.fromfunction(lambda i, j: i,
            (info.format.pixels_per_column,
                info.format.columns_per_frame), dtype=int)
    column_layer = np.fromfunction(lambda i, j: j,
            (info.format.pixels_per_column,
                info.format.columns_per_frame), dtype=int)
    column_layer_staggered = destagger(info, column_layer,
            inverse=True)

    def save_iter():
        nonlocal field_names, field_fmts
        try:
            for idx, scan in enumerate(scan_iter):

                # Initialize the field names for csv header
                if not field_names or not field_fmts:
                    field_names, field_fmts = get_fields_info(scan)

                # Copy per-column timestamps and measurement_ids for each beam
                timestamps = np.tile(scan.timestamp, (scan.h, 1))
                measurement_ids = np.tile(scan.measurement_id, (scan.h, 1))

                # Grab channel data
                fields_values = [scan.field(ch) for ch in scan.fields]

                frame = np.dstack((timestamps, row_layer, column_layer_staggered,
                    measurement_ids, *fields_values))

                # Output points in "image" vs. staggered order
                frame = destagger(info, frame)

                # Destagger XYZ separately since it has a different type
                xyz = xyzlut(scan.field(ChanField.RANGE))
                xyz_destaggered = destagger(info, xyz)

                if dual:
                    xyz2 = xyzlut(scan.field(ChanField.RANGE2))
                    xyz2_destaggered = destagger(info, xyz2)

                    # Get all data as one H x W x num fields int64 array for savetxt()
                    frame = np.dstack(tuple(map(lambda x: x.astype(object),
                        (frame, xyz_destaggered, xyz2_destaggered))))

                else:
                    # Get all data as one H x W x num fields int64 array for savetxt()
                    frame = np.dstack(tuple(map(lambda x: x.astype(object),
                        (frame, xyz_destaggered))))

                frame_colmajor = np.swapaxes(frame, 0, 1)

                # Write csv out to file
                csv_path = f"{filename}_{idx}.csv"
                print(f'write frame index #{idx}, to file: {csv_path}')

                if os.path.isfile(csv_path) and not overwrite:
                    print(_file_exists_error(csv_path))
                    exit(1)

                header = '\n'.join([f'frame num: {idx}', field_names])

                np.savetxt(csv_path,
                        frame_colmajor.reshape(-1, frame.shape[2]),
                        fmt=field_fmts,
                        delimiter=',',
                        header=header)

                yield scan
        except (KeyboardInterrupt, StopIteration):
            pass

    return save_iter()


# Determines the filename to use
def determine_filename(prefix: str, dir: str, filename: str, extension: str, info: SensorInfo):
    outpath = Path.cwd()
    if dir:
        outpath = Path(dir)

    time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = f"{prefix}_" if prefix else prefix

    if filename != "":
        filename = str(outpath / f"{prefix}{filename}")
    else:
        filename = str(outpath / f"{prefix}{info.prod_line}_{info.fw_rev}_{info.mode}_{time_str}{extension}")

    return filename


# Creates path to file if any folders in the chain are missing
def create_directories_if_missing(filename: str):
    outpath = Path(filename).parents[0]
    if not outpath.is_dir():
        outpath.mkdir(parents=True)


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.option('-r', '--raw', is_flag=True, default=False, help="Save in raw mode, "
              "where LidarPackets and ImuPackets from compatible sources are saved directly. "
              "This mode does not preserve LidarScan transformations performed by other commands "
              "in a multi-command chain. This mode preserves LEGACY ImuPackets.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_save_bag(ctx: SourceCommandContext, prefix: str, dir: str, filename: str,
                    raw: bool, overwrite: bool, **kwargs) -> None:
    """Save source as a packet rosbag."""
    if raw:
        _ = source_to_bag_iter(ctx.scan_source, ctx.scan_source.metadata, save_source_packets=True,  # type: ignore
                           prefix=prefix, dir=dir, filename=filename, overwrite=overwrite)
    else:
        ctx.scan_iter = source_to_bag_iter(ctx.scan_iter, ctx.scan_source.metadata,  # type: ignore
                                           prefix=prefix, dir=dir, filename=filename,
                                           overwrite=overwrite)


def source_to_bag_iter(scans: Union[ScanSource, Iterator[LidarScan]], info: SensorInfo,
                       sensor_idx: int = 0, save_source_packets: bool = False, prefix: str = "",
                       dir: str = "", filename: str = "", overwrite: bool = False) -> Iterator[LidarScan]:
    """Create a ROSBAG saving iterator from a LidarScan iterator

    Requires the active ROS environment or ROS-less rospy/rosbag python
    modules installed. See error message for details.

    If save_source_packets is selected, the raw packets from scans (which must be a compatible
    PacketSource - PCAP, BAG, or Live Sensor) are saved directly.

    Otherwise, each LidarScan in scans is deparsed into UDP packets and saved.
    """

    try:
        import ouster.sdk.pcap as pcap  # noqa: F401
    except ImportError:
        raise click.ClickException("Please verify that libpcap is installed")

    # Check that ROS imports are available
    import_rosbag_modules(raise_on_fail=True)
    from ouster.sdk.bag import PacketMsg  # type: ignore
    import rosbag  # type: ignore
    import rospy  # type: ignore

    # Build filename
    filename = determine_filename(filename=filename, info=info, extension=".bag", prefix=prefix, dir=dir)

    if os.path.isfile(filename) and not overwrite:
        print(_file_exists_error(filename))
        exit(1)

    create_directories_if_missing(filename)

    lidar_topic = "/os_node/lidar_packets"
    imu_topic = "/os_node/imu_packets"

    click.echo(f"Saving ROSBAG file at {filename}")

    if not save_source_packets:
        def save_iter():
            try:
                with rosbag.Bag(filename, 'w') as outbag:
                    for scan in scans:
                        packets = scan_to_packets(scan, info)
                        for packet in packets:
                            ts = rospy.Time.from_sec(packet.capture_timestamp)
                            msg = PacketMsg(buf=packet._data.tobytes())
                            if isinstance(packet, LidarPacket):
                                outbag.write(lidar_topic, msg, ts)
                            elif isinstance(packet, ImuPacket):
                                outbag.write(imu_topic, msg, ts)
                        yield scan
            except (KeyboardInterrupt, StopIteration):
                pass

        return save_iter()
    else:
        scan_source = None
        if isinstance(scans, ScanSourceAdapter):
            if isinstance(scans._scan_source, SensorScanSource):
                scan_source = scans._scan_source._scans  # type: ignore
            elif isinstance(scans._scan_source, PcapScanSource):
                scan_source = scans._scan_source  # type: ignore

        if scan_source is None:
            # [kk] TODO: Only single-source via ScanSourceAdapter is currently supported.
            # Revisit when implmenting multi source in CLI
            raise click.exceptions.BadParameter("Saving in -r/--raw mode is not supported with "
                                                "the current source type.")

        # replace ScanSource's packetsource with BagRecordingPacketSource
        scan_source._source = BagRecordingPacketSource(
            scan_source._source, filename, lidar_topic=lidar_topic, imu_topic=imu_topic
        )

        return scans  # type: ignore


class SourceSaveCommand(click.Command):
    """Generalizes ouster-cli source <> save <outputfile>
    """

    # Map from output type to a save implementation function
    implementations = {
        OusterIoType.OSF: source_save_osf,
        OusterIoType.PCAP: source_save_pcap,
        OusterIoType.CSV: source_save_csv,
        OusterIoType.BAG: source_save_bag,
    }

    def __init__(self, *args, **kwargs):
        kwargs['add_help_option'] = True
        super().__init__(*args, **kwargs)
        self.update_help()
        self.update_params()

    def update_help(self):
        help_str = "Save to an "
        help_str += _join_with_conjunction([k.name.upper() for k in self.implementations.keys()])
        help_str += " with the given filename. If only an extension is provided, the file is named automatically."
        self.help = help_str

    def update_params(self):
        # Add click options/parameters from save implementation commands
        param_mapping = {}
        for (iotype, cmd) in self.implementations.items():
            for p in cmd.params:
                if p.name in param_mapping.keys():
                    param_mapping[p.name][1].append(iotype)
                else:
                    param_mapping[p.name] = (p, [iotype])

        # Prefix options/parameters with name of the output iotype
        self.params = []
        for (_, (param, iotypes)) in param_mapping.items():
            if len(iotypes) < len(self.implementations):
                help_prefix = "|".join([k.name.upper() for k in iotypes])
                help_prefix = f"[{help_prefix}]:"
                # Click calls this init function multiple times on --help.
                # Check that the help string has not already been prepended with param.help
                if help_prefix not in param.help:
                    param.help = f"{help_prefix} {param.help}"
            self.params.append(param)

    def get_help(self, *args, **kwargs):
        # Update help text to capture changes from lazily loaded save implementations
        self.update_help()
        return super().get_help(*args, **kwargs)

    def get_params(self, *args, **kwargs):
        # Update params to capture changes from lazily loaded save implementations
        self.update_params()
        return super().get_params(*args, **kwargs)

    def invoke(self, ctx, *args):
        output_name = ctx.params.get('filename')
        output_format = ""

        split = os.path.splitext(output_name)
        if split[0][0] == '.' and split[1] == "":
            output_format = split[0].replace(".", "")
            ctx.params["filename"] = ""
        elif split[1] == "":
            click.echo("Error: Must provide a filename with an extension.")
            exit(2)
        else:
            output_format = split[1].replace(".", "")

        # Ensure the file extension is present and a valid one
        supported_formats = [iotype.name.upper() for iotype in self.implementations.keys()]
        if output_format.upper() not in supported_formats:
            string = f"Error: Invalid file extension. '.{output_format.lower()}' is not one of "
            string += _join_with_conjunction([f".{x.lower()}" for x in supported_formats])

            click.echo(string + ".")
            exit(2)

        ctx.params["format"] = output_format
        output_type = io_type_from_extension(f" .{output_format}")
        convert_command = self.implementations[output_type]
        if CliArgs().has_any_of(ctx.help_option_names):
            click.echo(convert_command.get_help(ctx))
        else:
            try:
                return ctx.forward(convert_command)
            except TypeError:
                if len(ctx.args) > 0:
                    raise SourceArgsException(ctx)
