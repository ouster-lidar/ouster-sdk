from ouster.cli.core.cli_args import CliArgs
import click
import time
from datetime import datetime
from pathlib import Path
import numpy as np
from typing import (Tuple, List, Iterator, Optional, Union)
from ouster.sdk.client import (get_field_types, first_valid_packet_ts,
                           Sensor, UDPProfileLidar,
                           LidarScan, ChanField, XYZLut,
                           ScanSource, destagger, SensorInfo,
                           LidarPacket, ImuPacket)
import ouster.cli.core.pcap
import ouster.cli.core.sensor
from ouster.sdk import osf
from ouster.sdk.pcap import RecordingPacketSource
from ouster.sdk.util import scan_to_packets  # type: ignore
from ouster.sdk.pcap.pcap import MTU_SIZE, Pcap
import ouster.sdk.pcap._pcap as _pcap
from .source_util import (SourceCommandContext,
                          SourceCommandType,
                          source_multicommand,
                          _join_with_conjunction,
                          import_rosbag_modules)
from .io_type import (extension_from_io_type,
                      io_type_from_extension,
                      OusterIoType)
from contextlib import closing


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('--chunk-size', default=0, help="Split output by size (MB)")
@click.option('-r', '--raw', is_flag=True, default=False, help="Save in raw mode, "
              "where LidarPackets and ImuPackets from compatible sources are saved directly. "
              "This mode does not preserve LidarScan transformations performed by other commands "
              "in a multi-command chain. This mode preserves LEGACY ImuPackets.")
@click.pass_context
@source_multicommand(type=SourceCommandType.UNTYPED)
def source_save_pcap(ctx: SourceCommandContext, prefix: str, dir: str,
                     chunk_size: int, raw: bool, **kwargs) -> None:
    """Save source as a PCAP"""
    scan_source = ctx.scan_source
    scans = ctx.scan_iter
    info = scan_source.metadata  # type: ignore

    # Output directory
    outpath = Path.cwd()
    if dir:
        outpath = Path(dir)
        if not outpath.is_dir():
            outpath.mkdir(parents=True)

    # Automatic file naming
    time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = f"{prefix}_" if prefix else prefix
    filename = str(outpath / f"{prefix}{info.prod_line}_{info.fw_rev}_{info.mode}_{time_str}")

    packetsource = getattr(scan_source, "_source", None)
    packetsource = (packetsource if
                    type(packetsource) is Pcap or
                    type(packetsource) is Sensor
                    else None)

    click.echo(f"Saving PCAP file at {filename}.pcap")
    click.echo(f"Saving metadata json at {filename}.json")

    # Save metadata as json
    with open(f"{filename}.json", 'w') as f:
        f.write(info.updated_metadata_string())

    if raw:
        if type(ctx.scan_source) not in [Pcap, Sensor]:
            raise click.exceptions.BadParameter("Saving in --raw mode is not supported with the current source type.")

        if len(ctx.invoked_command_names) != 1:
            click.echo("Warning: Saving pcap without -r/--rebuild will drop any LidarScan "
                       "transformations perfomed by other commands in this multi-command chain: "
                       f"{', '.join([c for c in ctx.invoked_command_names if c != 'save'])}.")

        # Replace ScanSource's packetsource with RecordingPacketSource
        ctx.scan_source._source = RecordingPacketSource(  # type: ignore
            packetsource, "", n_frames=None,  # type: ignore
            prefix=prefix, chunk_size=chunk_size,
            lidar_port=info.udp_port_lidar, imu_port=info.udp_port_imu
        )

    else:
        click.echo("Warning: Saving pcap without -r/--raw will not save LEGACY IMU packets.")

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
        ctx.scan_iter = save_iter()


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.pass_context
@source_multicommand(type=SourceCommandType.UNTYPED)
def source_save_osf(ctx: SourceCommandContext, prefix: str, dir: str, **kwargs) -> None:
    """Save source as an OSF"""
    scans = ctx.scan_iter
    info = ctx.scan_source.metadata  # type: ignore

    # Output directory
    outpath = Path.cwd()
    if dir:
        outpath = Path(dir)
        if not outpath.is_dir():
            outpath.mkdir(parents=True)

    # Automatic file naming
    time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = f"{prefix}_" if prefix else prefix
    filename = str(outpath / f"{prefix}{info.prod_line}_{info.fw_rev}_{info.mode}_{time_str}")

    # Initialize osf writer
    osf_writer = osf.Writer(f"{filename}.osf")
    osf_lidar_meta = osf.LidarSensor(metadata_json=info.original_string())
    lidar_id = osf_writer.addMetadata(osf_lidar_meta)

    click.echo(f"Saving OSF file at {filename}.osf")

    first_scan = next(scans)  # type: ignore
    field_types = get_field_types(first_scan)  # type: ignore
    osf_stream = osf.LidarScanStream(osf_writer, lidar_id, field_types)

    # TODO: extrinsics still need to be plugged in here -- Tim T.

    def write_osf(scan: LidarScan):
        # Set OSF timestamp to the timestamp of the first valid column
        ts = first_valid_packet_ts(scan)
        if ts:
            osf_stream.save(ts, scan)

    write_osf(first_scan)

    def save_iter():
        try:
            with closing(osf_writer):
                for scan in scans:
                    # Drop invalid lidarscans
                    if np.any(scan.status):
                        write_osf(scan)
                    yield scan
        except (KeyboardInterrupt, StopIteration):
            pass
    ctx.scan_iter = save_iter()


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('-g', '--gzip', is_flag=True, default=False,
              help="Compress with gzip. Can be loaded with numpy.loadtxt.")
@click.pass_context
@source_multicommand(type=SourceCommandType.UNTYPED)
def source_save_csv(ctx: SourceCommandContext, prefix: str,
                    dir: str, gzip: bool, **kwargs) -> None:
    """Save source as one CSV file per LidarScan."""
    ctx.scan_iter = source_to_csv_iter(ctx.scan_iter, ctx.scan_source.metadata,  # type: ignore
                                       prefix=prefix, dir=dir, gzip=gzip)


def source_to_csv_iter(scan_iter: Iterator[LidarScan], info: SensorInfo,
                       prefix: str = "", dir: Optional[str] = None,
                       gzip: bool = False) -> Iterator[LidarScan]:
    """Create a CSV saving iterator from a LidarScan iterator

    The number of saved lines per csv file is always H x W, which corresponds to
    a full 2D image representation of a lidar scan.

    Each line in a csv file is (for DUAL profile):

        TIMESTAMP (ns), RANGE (mm), RANGE2 (mm), SIGNAL (photons),
            SIGNAL2 (photons), REFLECTIVITY (%), REFLECTIVITY2 (%),
            NEAR_IR (photons), X (m), Y (m), Z (m), X2 (m), Y2 (m), Z2(m),
            MEASUREMENT_ID, ROW, COLUMN

    If ``--gzip`` is specified, the file is automatically saved in
    compressed gzip format. :func:`.numpy.loadtxt` can be used to read gzipped
    files transparently back to :class:`.numpy.ndarray`.
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
    ext = "gz" if gzip else "csv"
    outpath = Path.cwd()
    if dir:
        outpath = Path(dir)
        if not outpath.is_dir():
            outpath.mkdir(parents=True)

    time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = f"{prefix}_" if prefix else prefix
    filename = str(outpath / f"{prefix}{info.prod_line}_{info.fw_rev}_{info.mode}_{time_str}")

    click.echo(f"Saving CSV file at {filename}.{ext}")

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
                csv_path = f"{filename}_{idx}.{ext}"
                print(f'write frame index #{idx}, to file: {csv_path}')

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


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('-r', '--raw', is_flag=True, default=False, help="Save in raw mode, "
              "where LidarPackets and ImuPackets from compatible sources are saved directly. "
              "This mode does not preserve LidarScan transformations performed by other commands "
              "in a multi-command chain. This mode preserves LEGACY ImuPackets.")
@click.pass_context
@source_multicommand(type=SourceCommandType.UNTYPED)
def source_save_bag(ctx: SourceCommandContext, prefix: str, dir: str,
                    raw: bool, **kwargs) -> None:
    """Save source as a packet rosbag."""
    if raw:
        if type(ctx.scan_source) not in [Pcap, Sensor]:
            raise click.exceptions.BadParameter("Saving in --raw mode is not supported with the current source type.")
        _ = source_to_bag_iter(ctx.scan_source, ctx.scan_source.metadata, save_source_packets=True,  # type: ignore
                           prefix=prefix, dir=dir)
    else:
        ctx.scan_iter = source_to_bag_iter(ctx.scan_iter, ctx.scan_source.metadata,  # type: ignore
                                           prefix=prefix, dir=dir)


def source_to_bag_iter(scans: Union[ScanSource, Iterator[LidarScan]], info: SensorInfo,
                       save_source_packets: bool = False, prefix: str = "",
                       dir: Optional[str] = None) -> Iterator[LidarScan]:
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
    outpath = Path.cwd()
    if dir:
        outpath = Path(dir)
        if not outpath.is_dir():
            outpath.mkdir(parents=True)

    time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = f"{prefix}_" if prefix else prefix
    filename = str(outpath / f"{prefix}{info.prod_line}_{info.fw_rev}_{info.mode}_{time_str}.bag")

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
        packet_source = getattr(scans, "_source", None)
        packet_source = (packet_source if
                         type(packet_source) is Pcap or
                         type(packet_source) is Sensor
                         else None)
        if packet_source is None:
            raise RuntimeError("Error source_to_bag_iter: incompatible scans object for saving raw packets.")

        def saving_packet_iter():
            try:
                with rosbag.Bag(filename, 'w') as outbag:
                    for packet in packet_source:
                        ts = rospy.Time.from_sec(packet.capture_timestamp)
                        msg = PacketMsg(buf=packet._data.tobytes())
                        if isinstance(packet, LidarPacket):
                            outbag.write(lidar_topic, msg, ts)
                        elif isinstance(packet, ImuPacket):
                            outbag.write(imu_topic, msg, ts)
                    yield packet
            except (KeyboardInterrupt, StopIteration):
                pass

        scans._source = saving_packet_iter()  # type: ignore
        return scans  # type: ignore


class SourceSaveCommand(click.Command):
    """Generalizes ouster-cli source <> save <outputfile>
    """

    # Map from output type to a conversion function
    conversions = {
        OusterIoType.OSF: source_save_osf,
        OusterIoType.PCAP: source_save_pcap,
        OusterIoType.CSV: source_save_csv,
        OusterIoType.BAG: source_save_bag,
    }

    help_str = "Save to "
    for idx, iotype in enumerate(conversions.keys()):
        if idx == len(conversions) - 1:
            help_str += f"or {iotype.name.upper()}."
        else:
            help_str += f"{iotype.name.upper()}, "

    def __init__(self, *args, **kwargs):
        kwargs['add_help_option'] = True
        kwargs['help'] = self.help_str
        super().__init__(*args, **kwargs)

        # Build list of supported formats
        supported_formats = []
        for iotype in self.conversions.keys():
            supported_formats.append(iotype.name.upper())
        click.option('-f', '--format', type=click.Choice(supported_formats, case_sensitive=False), default='OSF',
                     help="Output file format. Default: OSF")(self)

        # Add click options/parameters from conversion commands
        param_mapping = {}
        for (iotype, cmd) in self.conversions.items():
            for p in cmd.params:
                if p.name in param_mapping.keys():
                    param_mapping[p.name][1].append(iotype)
                else:
                    param_mapping[p.name] = (p, [iotype])

        # Prefix options/parameters with name of the output iotype
        for (_, (param, iotypes)) in param_mapping.items():
            if len(iotypes) < len(self.conversions):
                help_prefix = ""
                for idx, iotype in enumerate(iotypes):
                    help_prefix += iotype.name.upper()
                    if idx != (len(iotypes) - 1):
                        help_prefix += "|"
                # Click calls this init function multiple times on --help.
                # Check that the help string has not already been prepended with param.help
                if help_prefix not in param.help:
                    param.help = f"[{help_prefix}]: {param.help}"
            self.params.append(param)

    def get_output_type_file_extensions_str(self):
        exts = sorted(
            [extension_from_io_type(source_type) for source_type in self.conversions.keys()]
        )
        return _join_with_conjunction(exts)

    def invoke(self, ctx, *args):
        output_format = ctx.params.get('format')
        output_type = io_type_from_extension(f" .{output_format}")
        convert_command = self.conversions[output_type]
        if CliArgs().has_any_of(ctx.help_option_names):
            click.echo(convert_command.get_help(ctx))
        else:
            try:
                return ctx.forward(convert_command)
            except TypeError:
                if len(ctx.args) > 0:
                    raise ouster.cli.core.SourceArgsException(ctx)
