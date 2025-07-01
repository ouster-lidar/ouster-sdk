from ouster.cli.core.cli_args import CliArgs
import atexit
import click
import os
import time
from datetime import datetime
from pathlib import Path
import numpy as np
from typing import (cast, Dict, Union, Tuple, List, Iterator, Iterable, Optional)
from ouster.cli.core import SourceArgsException  # type: ignore[attr-defined]
from ouster.sdk.core import (UDPProfileLidar, LidarScan, ChanField, XYZLut,
                             destagger, SensorInfo, LidarPacket, ImuPacket,
                             PacketSource)
from ouster.sdk import osf, open_packet_source
from ouster.sdk.core.io_types import (io_type_from_extension, OusterIoType)
from ouster.sdk.util import scan_to_packets  # type: ignore
from ouster.sdk.pcap.pcap import MTU_SIZE
import ouster.sdk._bindings.pcap as _pcap
from .source_util import (SourceCommandContext,
                          SourceCommandType,
                          source_multicommand,
                          _join_with_conjunction)
from contextlib import closing
from .source_bag import _source_to_bag_iter  # type: ignore[attr-defined]


_file_exists_error = lambda filename: (f"Error: File '{filename}' already exists. Add --overwrite "
                                       "flag to overwrite and continue anyways.")


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.option('--duration', '-D', default=None, type=float, help="Duration to record")
@click.option('--ros2', is_flag=True, default=False, help="If true, save this as a ROS2 bag file.")
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED)
def source_save_raw(ctx: SourceCommandContext, prefix: str, dir: str, filename: str,
                    overwrite: bool, duration: Optional[float], ros2: bool, **kwargs) -> None:
    """ Save raw packets from the source. Does not support chaining. Supports Bag and PCAP formats."""
    uri = ctx.source_uri
    if uri is None:
        raise RuntimeError("Unexpected condition")
    extension = ""
    split = os.path.splitext(filename)
    if split[0][0] == '.' and split[1] == "":
        extension = split[0].replace(".", "")
        filename = ""
    elif split[1] == "":
        click.echo("Error: Must provide a filename with an extension.")
        exit(2)
    else:
        extension = split[1].replace(".", "")

    if extension != "pcap" and extension != "bag":
        raise click.exceptions.BadParameter(f"Cannot save raw file of type {extension}")

    # Finally open the appropriate packet source
    packets: PacketSource
    packets = open_packet_source(uri, **ctx.source_options)  # type: ignore

    if extension == "pcap":
        save_pcap_impl(packets, filename, prefix, dir, raw=True, overwrite=overwrite,
                       metadata=packets.sensor_info, duration=duration)
    elif extension == "bag":
        _source_to_bag_iter(packets, raw=True, metadata=packets.sensor_info,
                           prefix=prefix, dir=dir, filename=filename,
                           overwrite=overwrite, duration=duration, ros2=ros2)


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
def source_save_pcap(ctx: SourceCommandContext, prefix: str, dir: str, filename: str,
                     overwrite: bool, **kwargs) -> None:
    """Save source as a PCAP"""
    if ctx.scan_iter is None or ctx.scan_source is None:
        raise RuntimeError("unexpected condition")

    ctx.scan_iter = save_pcap_impl(ctx.scan_iter, filename, prefix, dir, False, overwrite, ctx.scan_source.sensor_info)


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
@click.option("--compression-level", default=1, help="Specifies the level of compression for OSF files. Higher values "
    "are slower but more space-efficient; lower values are faster but less space-efficient.", type=click.IntRange(0, 9))
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_save_osf(ctx: SourceCommandContext, prefix: str, dir: str, filename: str,
                    overwrite: bool, ts: str, continue_anyways: bool, compression_level: int, **kwargs) -> None:
    """Save source as an OSF"""
    scans = ctx.scan_iter
    info = ctx.scan_source.sensor_info  # type: ignore

    # Automatic file naming
    filename = determine_filename(filename=filename, info=info[0], extension=".osf", prefix=prefix, dir=dir)

    create_directories_if_missing(filename)

    click.echo(f"Saving OSF file at {filename}")

    if os.path.isfile(filename) and not overwrite:
        click.echo(_file_exists_error(filename))
        exit(1)

    # Initialize osf writer
    osf_writer = osf.AsyncWriter(filename, info, [], 0, osf.Encoder(osf.PngLidarScanEncoder(compression_level)))

    wrote_scans = False
    dropped_scans = 0
    last_ts = [0] * len(info)

    # returns false if we should stop recording
    def write_osf(scan: LidarScan, index: int):
        nonlocal wrote_scans, last_ts, dropped_scans
        # Set OSF timestamp to the timestamp of the first valid column
        scan_ts = scan.get_first_valid_packet_timestamp() if ts == "packet" \
            else scan.get_first_valid_column_timestamp()
        if scan_ts:
            if scan_ts < last_ts[index]:
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
            osf_writer.save(index, scan, scan_ts)
            last_ts[index] = scan_ts
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

    saved = False

    def save_iter():
        try:
            # only save the first loop
            nonlocal saved
            if saved:
                for s in scans():
                    yield s
                return

            with closing(osf_writer):
                stop = False
                for s in scans():
                    for index, scan in enumerate(s):
                        if scan is None:
                            continue
                        # Drop invalid lidarscans
                        if not stop and np.any(scan.status):
                            if not write_osf(scan, index):
                                stop = True
                    yield s
        except (KeyboardInterrupt):
            pass
        except (ValueError):
            ctx.terminate_evt.set()
        finally:
            saved = True

    ctx.scan_iter = save_iter  # type: ignore

    def handle_termination():
        osf_writer.close()
        if dropped_scans > 0:
            if ts == "lidar":
                click.echo(f"WARNING: Dropped {dropped_scans} scans because missing or decreasing timestamps.")
            else:
                click.echo(f"WARNING: Dropped {dropped_scans} scans because missing or decreasing "
                           "packet timestamps. Try with `--ts lidar` instead.")
        if not wrote_scans:
            click.echo("WARNING: No scans saved.")
    atexit.register(handle_termination)


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
    ctx.scan_iter = source_to_csv_iter(ctx.scan_iter, ctx.scan_source.sensor_info,  # type: ignore
                                       prefix=prefix, dir=dir, filename=filename,
                                       overwrite=overwrite)


# [doc-stag-pcap-to-csv]
def source_to_csv_iter(scan_iter: Iterator[List[Optional[LidarScan]]], infos: List[SensorInfo],
                       prefix: str = "", dir: str = "", overwrite: bool = True,
                       filename: str = "") -> Iterable[List[Optional[LidarScan]]]:
    """Create a CSV saving iterator from a LidarScan iterator

    The number of saved lines per csv file is always H x W, which corresponds to
    a full 2D image representation of a lidar scan.

    Each line in a csv file is (for DUAL profile):

        TIMESTAMP (ns), RANGE (mm), RANGE2 (mm), SIGNAL (photons),
            SIGNAL2 (photons), REFLECTIVITY (%), REFLECTIVITY2 (%),
            NEAR_IR (photons), X (m), Y (m), Z (m), X2 (m), Y2 (m), Z2(m),
            MEASUREMENT_ID, ROW, COLUMN
    """

    dual_formats = [UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL,
                    UDPProfileLidar.PROFILE_LIDAR_FUSA_RNG15_RFL8_NIR8_DUAL]
    for info in infos:
        if info.format.udp_profile_lidar in dual_formats:
            print("Note: You've selected to convert a dual returns pcap to CSV. Each row "
                  "will represent a single pixel, so that both returns for that pixel will "
                  "be on a single row. As this is an example we provide for getting "
                  "started, we realize that you may have conversion needs which are not met "
                  "by this function. You can find the source code on the Python SDK "
                  "documentation website to modify it for your own needs.")
            break

    # Build filenames
    filenames = []
    for info in infos:
        name = determine_filename(filename=filename, info=info, extension=".csv", prefix=prefix, dir=dir)

        name = name[0:-4]  # remove extension
        filenames.append(name)
        click.echo(f"Saving CSV files at {name}_XXX.csv")

    create_directories_if_missing(filenames[0])

    # Construct csv header and data format
    def get_fields_info(scan: LidarScan) -> Tuple[str, List[str]]:
        field_names = 'TIMESTAMP (ns), ROW, DESTAGGERED IMAGE COLUMN, MEASUREMENT_ID'
        field_fmts = ['%d'] * 4
        dual = ChanField.RANGE2 in scan.fields
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

    field_names: Dict[int, str] = {}
    field_fmts: Dict[int, List[str]] = {}

    # {recompute xyzlut to save computation in a loop
    xyzlut = []
    row_layer = []
    column_layer_staggered = []
    for info in infos:
        xyzlut.append(XYZLut(info))

        row_layer.append(np.fromfunction(lambda i, j: i,
                (info.format.pixels_per_column,
                    info.format.columns_per_frame), dtype=int))
        column_layer = np.fromfunction(lambda i, j: j,
                (info.format.pixels_per_column,
                    info.format.columns_per_frame), dtype=int)
        column_layer_staggered.append(destagger(info, column_layer,
                inverse=True))

    saved = False

    def save_iter():
        nonlocal field_names, field_fmts, saved
        try:
            if saved:
                for scan in scan_iter():
                    yield scan
                return
            for idx, scans in enumerate(scan_iter()):
                for lidar_idx, scan in enumerate(scans):
                    if scan is None:
                        continue

                    # Initialize the field names for csv header
                    if lidar_idx not in field_names or lidar_idx not in field_fmts:
                        field_names[lidar_idx], field_fmts[lidar_idx] = get_fields_info(scan)

                    # Copy per-column timestamps and measurement_ids for each beam
                    timestamps = np.tile(scan.timestamp, (scan.h, 1))
                    measurement_ids = np.tile(scan.measurement_id, (scan.h, 1))

                    # Grab channel data
                    fields_values = [scan.field(ch) for ch in scan.fields]

                    frame = np.dstack((timestamps, row_layer[lidar_idx], column_layer_staggered[lidar_idx],
                        measurement_ids, *fields_values))

                    # Output points in "image" vs. staggered order
                    frame = destagger(info, frame)

                    # Destagger XYZ separately since it has a different type
                    xyz = xyzlut[lidar_idx](scan.field(ChanField.RANGE))
                    xyz_destaggered = destagger(info, xyz)

                    if ChanField.RANGE2 in scan.fields:
                        xyz2 = xyzlut[lidar_idx](scan.field(ChanField.RANGE2))
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
                    csv_path = f"{filenames[lidar_idx]}_{idx}.csv"
                    print(f'write frame index #{idx}, to file: {csv_path}')

                    if os.path.isfile(csv_path) and not overwrite:
                        print(_file_exists_error(csv_path))
                        exit(1)

                    header = '\n'.join([f'frame num: {idx}', field_names[lidar_idx]])

                    np.savetxt(csv_path,
                            frame_colmajor.reshape(-1, frame.shape[2]),
                            fmt=field_fmts[lidar_idx],
                            delimiter=',',
                            header=header)

                yield scan
        except (KeyboardInterrupt, StopIteration):
            pass
        finally:
            saved = True

    # type ignored because generators are tricky to mypy
    return save_iter  # type: ignore
# [doc-etag-pcap-to-csv]


@click.command(context_settings=dict(
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_save_png(ctx: SourceCommandContext, prefix: str, dir: str,
                    filename: str, overwrite: bool, **kwargs) -> None:
    """Save scan source as a series of png files per LidarScan per field (represented as 8-bit)."""
    from PIL import Image
    from ouster.sdk.core.data import destagger
    scan_iter = ctx.scan_iter
    filename_no_ext = os.path.splitext(filename)[0]

    def normalize(image):
        min_val = np.min(image)
        max_val = np.max(image)
        if max_val == min_val:
            return np.zeros_like(image)
        return (image - min_val) / (max_val - min_val)

    def compose_path(scan: LidarScan, dir: str, prefix: str, field_name: str) -> str:
        output_path = f"{scan.sensor_info.sn}_{scan.frame_id}_{field_name}.png"
        if filename_no_ext:
            output_path = f"{filename_no_ext}_{output_path}"
        if prefix:
            output_path = f"{prefix}_{output_path}"
        if dir:
            output_path = os.path.join(dir, output_path)
        return output_path

    def save_field(scan: LidarScan, f: str):
        field_data = scan.field(f)
        img = destagger(scan.sensor_info, field_data)
        img = (normalize(img) * (2**8 - 1)).astype(np.uint8)
        pil_img = Image.fromarray(img)
        output_path = output_path = compose_path(scan, dir, prefix, f)
        if os.path.isfile(output_path) and not overwrite:
            print(_file_exists_error(output_path))
            exit(1)
        pil_img.save(output_path)

    def png_save_iter():
        for scans in scan_iter():
            for scan in scans:
                if scan:
                    for f in scan.fields:
                        save_field(scan, f)
            yield scans

    ctx.scan_iter = png_save_iter    # type: ignore


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
        filename = str(outpath / f"{prefix}{info.prod_line}_{info.fw_rev}_"
                       f"{info.config.lidar_mode}_{time_str}{extension}")

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
@click.option('--ros2', is_flag=True, default=False, help="If true, save this as a ROS2 bag file.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_save_bag(ctx: SourceCommandContext, prefix: str, dir: str, filename: str,
                    overwrite: bool, ros2: bool, **kwargs) -> None:
    """Save source as a packet rosbag."""
    if ctx.scan_iter is None or ctx.scan_source is None:
        raise RuntimeError("unexpected condition")

    ctx.scan_iter = _source_to_bag_iter(ctx.scan_iter, raw=False,
                                        prefix=prefix, dir=dir, filename=filename,
                                        overwrite=overwrite, metadata=ctx.scan_source.sensor_info, ros2=ros2)


def bag_save_metadata(outbag, infos):
    from std_msgs.msg import String  # type: ignore
    for idx, metadata in enumerate(infos):
        s = String()
        s.data = metadata.to_json_string()
        outbag.write(f"/ouster{idx}/metadata", s)


def save_pcap_impl(source: Union[Iterable[List[Optional[LidarScan]]], PacketSource],
                   filename, prefix, dir, raw, overwrite, metadata, duration = None):
    # Automatic file naming
    filename = determine_filename(filename=filename, info=metadata[0], extension=".pcap", prefix=prefix, dir=dir)

    create_directories_if_missing(filename)

    filename = filename[0:-5]  # remove extension

    # check for existing files
    pcap_filename = f"{filename}.pcap"
    if os.path.isfile(pcap_filename) and not overwrite:
        click.echo(_file_exists_error(pcap_filename))
        exit(1)

    for i, md in enumerate(metadata):
        fn = f"{filename}_{i}.json"
        if os.path.isfile(fn) and not overwrite:
            click.echo(_file_exists_error(fn))
            exit(1)

    # Save metadata as json
    for i, md in enumerate(metadata):
        fn = f"{filename}_{i}.json"
        with open(fn, 'w') as f:
            click.echo(f"Saving metadata json at {fn}")
            f.write(md.to_json_string())

    click.echo(f"Saving pcap file at {pcap_filename}")

    # Initialize pcap writer
    pcap_record_handle = _pcap.record_initialize(pcap_filename, MTU_SIZE, False)

    def save_packet(idx, packet, port):
        ts = packet.host_timestamp / 1e9 if packet.host_timestamp else time.time()
        _pcap.record_packet(pcap_record_handle, "127.0.0.1",
                            "127.0.0.1", port, port, packet.buf, ts)

    if raw:
        end_time = None
        try:
            source = cast(PacketSource, source)
            for idx, packet in source:
                if isinstance(packet, LidarPacket):
                    save_packet(idx, packet, metadata[idx].config.udp_port_lidar)
                elif isinstance(packet, ImuPacket):
                    save_packet(idx, packet, metadata[idx].config.udp_port_imu)

                if duration is not None:
                    if end_time is None:
                        end_time = packet.host_timestamp + duration * 1e9
                    if packet.host_timestamp > end_time:
                        break
        except (KeyboardInterrupt, StopIteration):
            pass
        finally:
            # Finish pcap_recording when this generator is garbage collected
            _pcap.record_uninitialize(pcap_record_handle)
    else:
        click.echo("Warning: Saving pcap without save_raw will not save LEGACY IMU packets.")

        def save_iter():
            try:
                # only save the first loop
                nonlocal pcap_record_handle
                if pcap_record_handle is None:
                    for c in source():
                        yield c
                    return
                for c in source():
                    for idx, scan in enumerate(c):
                        if scan is not None:
                            packets = scan_to_packets(scan, metadata[idx])
                            for packet in packets:
                                save_packet(idx, packet, metadata[idx].config.udp_port_lidar)
                    yield c
            except (KeyboardInterrupt, StopIteration):
                pass
            finally:
                # Finish pcap_recording when this generator is garbage collected
                if pcap_record_handle is not None:
                    _pcap.record_uninitialize(pcap_record_handle)
                    pcap_record_handle = None

        return save_iter


class SourceSaveCommand(click.Command):
    """Generalizes ouster-cli source <> save <outputfile>
    """

    # Map from output type to a save implementation function
    implementations = {
        OusterIoType.OSF: source_save_osf,
        OusterIoType.PCAP: source_save_pcap,
        OusterIoType.BAG: source_save_bag,
        OusterIoType.CSV: source_save_csv,
        OusterIoType.PNG: source_save_png
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
