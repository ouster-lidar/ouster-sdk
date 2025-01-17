#  type: ignore
import atexit
import click
import re
import sys
import importlib.metadata
import threading
import numpy as np
from itertools import islice
from ouster.cli.core import cli
from ouster.cli.core.cli_args import CliArgs
from ouster.cli.core.util import click_ro_file
from ouster.sdk import open_source, SourceURLException
from ouster.sdk.client import (LidarScan, SensorInfo, ImuPacket, Sensor,
                               PacketFormat, first_valid_packet_ts)
from ouster.sdk.client.core import ClientTimeout
from ouster.sdk.pcap import PcapDuplicatePortException
from ouster.sdk.util import resolve_metadata
from ouster.sdk.io_type import (extension_from_io_type, io_type, OusterIoType)
from ouster.sdk.osf import OsfScanSource
import ouster.cli.plugins.source_pcap as pcap_cli
import ouster.cli.plugins.source_osf as osf_cli
import ouster.cli.plugins.source_sensor as sensor_cli
from ouster.sdk.util.extrinsics import parse_extrinsics_from_string
import ouster.sdk.util.pose_util as pu
from typing import (Optional, Iterable, Tuple, Union)
from .source_save import (SourceSaveCommand, source_save_raw)
from .source_util import (CoupledTee,
                          SourceCommandContext,
                          SourceCommandCallback,
                          SourceCommandType,
                          source_multicommand,
                          _join_with_conjunction,
                          _nanos_to_string)
from .source_bag import bag_metadata, bag_info
_source_arg_name: str = 'source'


def is_ouster_mapping_installed():
    try:
        importlib.metadata.distribution('ouster-mapping')
        click.echo(click.style("Error: The ouster-mapping package is merged into the ouster-sdk package.\n"
                               "Run 'pip3 uninstall ouster-mapping' to uninstall it and avoid potential issues.",
                               fg='red'))
        sys.exit(1)
    except importlib.metadata.PackageNotFoundError:
        pass


@click.command()
@click.option("-p", "--pause", is_flag=True, help="Pause at first lidar scan")
@click.option("-e", "--on-eof", default='loop', type=click.Choice(['loop', 'stop', 'exit']),
              help="Loop, stop or exit after reaching end of file")
@click.option('-r',
              '--rate',
              default="1",
              help="Playback rate.",
              type=click.Choice(["0.25", "0.5", "0.75", "1", "1.5", "2", "3", "max"]))
@click.option("--pause-at",
              default=-1,
              help="Lidar Scan number to pause at")
@click.option("--accum-num",
              default=None,
              type=int,
              help="Accumulate up to this number of past scans for visualization. "
                   "Use <= 0 for unlimited. Defaults to 100 if --accum-every or --accum-every-m is set.")
@click.option("--accum-every",
              default=None,
              type=int,
              help="Add a new scan to the accumulator every this number of scans.")
@click.option("--accum-every-m",
              default=None,
              type=float,
              help="Add a new scan to the accumulator after this many meters of travel.")
@click.option("--map", "_map",
              is_flag=True,
              help="If set, add random points from every scan into an overall map for visualization. "
                   "Enabled if either --map-ratio or --map-size are set.")
@click.option("--map-ratio",
              default=None, type=float,
              help="Fraction of random points in every scan to add to overall map (0, 1]. [default: 0.01]")
@click.option("--map-size",
              default=None, type=int,
              help="Maximum number of points in overall map before discarding. [default: 1500000]")
@click.option("--global-map", default=None, type=str,
              help="A path to a ply file that represents the global map to display in the ouster-viz. "
                   "When using this option with the `localize` command it will replace the visualized global "
                   " map but it won't affect the map used during localization")
@click.option("--global-map-min-z", default=None, type=float,
              help="Filter out points below this value on the z-axis of the global map")
@click.option("--global-map-max-z", default=None, type=float,
              help="Filter out points above this value on the z-axis in the global map")
@click.option("--global-map-flatten", default=True, type=bool, show_default=True,
              help="Flatten the global map")
@click.option("--global-map-voxel-size", default=None, type=float,
              help="When set, the global map will be downsampled using the specified voxel size (meters)")
@click.option("--global-map-point-size", default=1.0, type=float, show_default=True,
              help="Set the point size of the the global map")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_viz(ctx: SourceCommandContext, pause: bool, on_eof: str, pause_at: int, accum_num: Optional[int],
               accum_every: Optional[int], accum_every_m: Optional[float], _map: bool, map_ratio: float,
               rate: str, map_size: int, global_map: Optional[str], global_map_min_z: Optional[float],
               global_map_max_z: Optional[float], global_map_flatten: bool,
               global_map_voxel_size: Optional[float], global_map_point_size: float) -> SourceCommandCallback:
    """Visualize LidarScans in a 3D viewer."""
    try:
        from ouster.sdk.viz import SimpleViz
    except ImportError as e:
        raise click.ClickException(str(e))

    source = ctx.scan_source

    # ugly workarounds ensue
    if on_eof == 'loop':
        # NOTE: setting it here instead of at open_source stage because we do not want to propagate
        #       the flag up to `source` command
        source._cycle = True

    if pause and pause_at == -1:
        pause_at = 0

    # Determine how to set the rate
    if rate == "max":
        rate = 0.0
    else:
        rate = float(rate)
    if source.is_live:
        if rate != 1.0:
            raise click.exceptions.UsageError("Can only set a rate of 1 for live sources")
        rate = None

    ctx.scan_iter, scans = CoupledTee.tee(ctx.scan_iter,
                                          terminate=ctx.terminate_evt)
    metadata = ctx.scan_source.metadata

    # build the accumulator
    if accum_every_m is not None and accum_every is not None:
        raise click.exceptions.UsageError("Can only provide one of --accum-every and --accum-every-m")

    if accum_num is not None and accum_num <= 0:
        accum_num = 1000000

    if accum_every_m is not None or accum_every is not None:
        if accum_num is None:
            accum_num = 100
    elif accum_num is None:
        accum_num = 0

    if accum_every_m is not None:
        accum_every = 0
    elif accum_every is not None:
        accum_every_m = 0.0
    else:
        accum_every = 0 if accum_num is None else 1
        accum_every_m = 0.0

    if map_ratio is not None or map_size is not None:
        _map = True
    map_ratio = 0.01 if map_ratio is None else map_ratio
    map_size = 1500000 if map_size is None else map_size

    if map_ratio > 1.0 or map_ratio <= 0.0:
        raise click.exceptions.UsageError("--map-ratio must be in the range (0, 1]")
    if map_size <= 0:
        raise click.exceptions.UsageError("--map-size must be greater than 0")

    def viz_thread_fn():
        sv = SimpleViz(
            metadata,
            rate=rate, pause_at=pause_at, on_eof=on_eof,
            accum_max_num=accum_num,
            accum_min_dist_num=accum_every,
            accum_min_dist_meters=accum_every_m,
            map_enabled=_map,
            map_select_ratio=map_ratio,
            map_max_points=map_size,
            title="Ouster Viz: " + ctx.source_uri
        )

        map_path = ctx.get("localization.map", None) if global_map is None else global_map

        if map_path is not None:
            import point_cloud_utils as pcu
            from ouster.sdk.viz import Cloud
            click.echo("Start loading global points into VIZ")
            pts = pcu.load_mesh_v(map_path)
            if global_map_min_z:
                pts = pts[pts[:, 2] >= global_map_min_z]
            if global_map_max_z:
                pts = pts[pts[:, 2] <= global_map_max_z]
            if global_map_voxel_size:
                pts = pcu.downsample_point_cloud_on_voxel_grid(
                        global_map_voxel_size, pts)
            if global_map_flatten:
                pts[:, 2] = 0
            cloud_xyz = Cloud(len(pts))
            cloud_xyz.set_xyz(pts)
            cloud_xyz.set_key(np.full(len(pts), 1))
            cloud_xyz.set_point_size(global_map_point_size)
            sv._viz.add(cloud_xyz)

        sv.run(scans)
        ctx.terminate_evt.set()

    if ctx.main_thread_fn is not None:
        raise RuntimeError(
            "A main-thread required function has already been set.")
    ctx.main_thread_fn = viz_thread_fn


# global to store slice argument for quick index
_last_slice = None


def extract_slice_indices(click_ctx: Optional[click.core.Context],
                          param: Optional[click.core.Argument], value: str):
    """Validate and extract slice indices of the form [start]:[stop][:step]."""
    matches = re.findall(r"^(?:(\d+(?:\.\d+)?)(h|min|s|ms)?)?"
                         r":(?:(\d+(?:\.\d+)?)(h|min|s|ms)?)?(?::(-?\d*))?$", value)  # noqa: W605

    if not matches or len(matches[0]) != 5:
        raise click.exceptions.BadParameter(
            "slice indices must be of the form [start]:[stop][:step]")

    multipliers = {'': 1, 'ms': 0.001, 's': 1.0, 'min': 60, 'h': 3600}
    m = matches[0]
    has_units = m[1] != "" or m[3] != ""
    has_decimals = (m[0] is not None and '.' in m[0]) or (m[0] is not None and '.' in m[2])
    frame_based = not (has_units or has_decimals)
    parsed_indices = [
        float(m[0]) * multipliers[m[1]] if m[0] != "" else None,
        float(m[2]) * multipliers[m[3]] if m[2] != "" else None,
        int(m[4]) if m[4] != "" else None]
    start, stop, step = parsed_indices[0], parsed_indices[1], parsed_indices[2]
    start = start if start is not None else 0
    # Check that indices are non-negative
    if any(i < 0 if i is not None else False for i in parsed_indices):
        raise click.exceptions.BadParameter(
            "slice indices must be non-negative")
    # Check that stop > start
    if (stop is not None) and (not stop > start):
        raise click.exceptions.BadParameter(
            "slice stop index must be greater than start")
    # Check that step > 1
    if (step is not None) and (not step > 0):
        raise click.exceptions.BadParameter(
            "slice step index must be greater than 0")

    # since some of the code logic depends on the type if float or int make sure
    # to apply the approprite the case
    type_caster = int if frame_based else float
    start = type_caster(start) if start is not None else start
    stop = type_caster(stop) if stop is not None else stop
    step = int(step) if step is not None else step
    # Store the argument so we can extract it for fast slicing if possible
    global _last_slice
    _last_slice = (start, stop, step)
    return start, stop, step, frame_based


def tslice(scans_iter, start, stop, step):
    start_time = None
    counter = 0
    for scan in scans_iter:
        scan_time = None
        for s in scan:
            if s:
                scan_time = first_valid_packet_ts(s)
                break
        if scan_time == 0 or scan_time is None:
            click.secho("WARNING: Scan missing packet timestamps. "
                        "Yielding scan in time slice anyways.", fg='yellow')
            yield scan
            continue
        scan_time = scan_time / 1e9
        if start_time is None:
            start_time = scan_time
        dt = scan_time - start_time
        if dt >= start:
            if not stop or dt <= stop:
                if not step or counter % step == 0:
                    yield scan
                counter = counter + 1
            else:
                return


@click.command()
@click.argument('indices', required=True, callback=extract_slice_indices)
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR)
def source_slice(ctx: SourceCommandContext,
                 indices: Tuple[Optional[int]]) -> SourceCommandCallback:
    """Slice LidarScans streamed from SOURCE. Use the form [start]:[stop][:step].
    Optionally can specify start and stop as times relative to the start of the file
    using the units h (hours), min (minutes), s (seconds), or ms (milliseconds).
    For example: 10s:20s:2"""

    start, stop, step, frame_based = indices
    slice_method = islice if frame_based else tslice
    scans_iter = ctx.scan_iter
    ctx.scan_iter = slice_method(scans_iter, start, stop, step)


def extract_clip_indices(click_ctx: Optional[click.core.Context],
                         param: Optional[click.core.Argument], value: str):
    """Validate and extract slice indices of the form [lower]:[upper]."""

    matches = re.findall(r"^(?:(\-?\d+(?:\.\d+)?)(mm|cm|dm|m)?)?"
                         r":(?:(\-?\d+(?:\.\d+)?)(mm|cm|dm|m)?)?$", value)

    if not matches or len(matches[0]) != 4:
        raise click.exceptions.BadParameter(
            "slice indices must be of the form [lower]:[upper]")

    multipliers = {'': 1, 'mm': 1, 'cm': 10, 'dm': 100, 'm': 1000}
    m = matches[0]
    parsed_indices = [
        float(m[0]) * multipliers[m[1]] if m[0] != "" else None,
        float(m[2]) * multipliers[m[3]] if m[2] != "" else None]
    start, stop = parsed_indices[0], parsed_indices[1]
    start = start if start is not None else 0
    # Check that stop >= start
    if (stop is not None) and (not stop >= start):
        raise click.exceptions.BadParameter(
            "clip `upper` value must be greater or equal to `lower`")
    return start, stop


@click.command
@click.argument('fields', required=True, type=str)
@click.argument('indices', required=True, callback=extract_clip_indices)
@click.option('--out-of-range-value', default=0, show_default=True,
              help="The value used when replacing out of range values")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR)
def source_clip(ctx: SourceCommandContext, fields: Optional[str],
                indices: Tuple[Optional[int]], out_of_range_value: int,
                **kwargs) -> None:
    """
    Constraints the range of values of the specified fields to the range [lower, upper] inclusive.
    Any value beyond this range is replaced with the out-of-range-value (default is zero).
    Use the form clip FIELDS [lower[u]]:[upper[u]]; where `FIELDS` is a comma separated list of fields
    names (no spaces) that this operation will be applied to, the `u` is an optional unit specifier;
    Supported units and their effects {mm: 1x, cm: 10x, dm: 100x, m: 1000x}.

    Usage example 1: `clip RANGE,RANGE2 :50m` would zero any RANGE values of the scan higher than 50
    meters. If the metric unit is not supplied the passed value will be used with no change, that is if
    a user passes :50 then it would be evaluated as 50 millimeters for RANGE values and merely as 50 units
    for other fields.

    Usage example 2: `clip RANGE 50m:50m` would only forward RANGE values of the scan that are
    exactly 50 meters.
    """
    import ouster.sdk.client.scan_ops as so

    scan_iter = ctx.scan_iter
    field_list = fields.strip().split(',')
    start, stop = indices
    start = start if start else 0
    stop = stop if stop else float('inf')   # NOTE: this probably should be sys.maxint

    def clip_iter():
        for scans in scan_iter:
            out = [None] * len(scans)
            for idx, scan in enumerate(scans):
                if scan:
                    result = LidarScan(scan)
                    so.clip(result, field_list, start, stop, out_of_range_value)
                    out[idx] = result
            yield out

    ctx.scan_iter = clip_iter()


@click.command
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def source_plumb(ctx: SourceCommandContext, click_ctx: click.core.Context) -> None:
    """Calculate the extrinsic matrix to align the sensor's Z-axis with the
    gravity vector using IMU data"""
    source_name = ctx.source_uri

    if source_name.endswith(".pcap"):
        try:
            import ouster.sdk.pcap as pcap
        except ImportError:
            raise click.ClickException("Please verify that libpcap is installed")

        try:
            meta = resolve_metadata(source_name)
            with open(meta, 'r') as json_file:
                info = SensorInfo(json_file.read())
            source = pcap.Pcap(source_name, info)
        except Exception as e:
            raise click.ClickException(f"Error reading pcap file: {e}")
    else:
        try:
            source = Sensor(source_name, 7502, 7503)
        except Exception as e:
            raise click.ClickException(f"Error connecting to sensor: {e}")
        info = source.metadata

    pf = PacketFormat(info)

    ax_sum = 0
    ay_sum = 0
    az_sum = 0
    count = 0

    for packet in source:
        if count >= 100:
            break

        if isinstance(packet, ImuPacket):
            ax_sum += pf.imu_la_x(packet.buf)
            ay_sum += pf.imu_la_y(packet.buf)
            az_sum += pf.imu_la_z(packet.buf)
            count += 1

    if count > 0:
        ax_avg = ax_sum / count
        ay_avg = ay_sum / count
        az_avg = az_sum / count
        click.echo(f"Average ax: {ax_avg}, ay: {ay_avg}, az: {az_avg}")
    else:
        click.echo(f"ERROR: No IMU packet found in the source {source_name}")
        return

    rotation_matrix = pu.get_rot_matrix_to_align_to_gravity(ax_avg, ay_avg, az_avg)
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    formatted_matrix = " ".join(
            " ".join(
                "0" if item == 0 else ("1" if item == 1 else f"{item:.8f}")
                for item in row)
            for row in transformation_matrix)

    click.echo("Transformation Matrix to Correct Sensor Orientation:")
    click.echo(transformation_matrix)
    click.echo("Flatten Transformation Matrix:")
    click.echo(formatted_matrix)


@click.command()
@click.option("-v", "--verbose", is_flag=True, help="Print out additional stats info.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_stats(ctx: SourceCommandContext, verbose: bool) -> SourceCommandCallback:
    """Calculate and output various statistics about the scans at this point in the pipeline."""
    windows = []
    for m in ctx.scan_source.metadata:
        windows.append(m.format.column_window)
    scans = ctx.scan_iter
    count = 0
    incomplete_count = 0
    start = None
    end = None
    incomplete_scans = []
    dimensions = {}

    def stats_iter():
        nonlocal count, incomplete_count, windows, verbose, start, end, incomplete_scans, dimensions
        ns_to_sec = 1.0 / 1000000000.0
        for l in scans:
            for i, scan in enumerate(l):
                if scan is None:
                    continue
                time = first_valid_packet_ts(scan)
                if time != 0.0:
                    if start is None or time < start:
                        start = time
                    if end is None or time > end:
                        end = time
                dimensions[(scan.w, scan.h)] = True
                if not scan.complete(windows[i]):
                    incomplete_count = incomplete_count + 1
                    if verbose:
                        incomplete_scans.append(f"    #{count} at {time * ns_to_sec}")
                count = count + 1
            yield l
    ctx.scan_iter = stats_iter()

    def exit_handler():
        nonlocal count, incomplete_count, start, end, incomplete_scans, dimensions
        ns_to_sec = 1.0 / 1000000000.0
        print("Scan Statistics:")
        print(f"  Count: {count}")
        dstring = ""
        for k in dimensions:
            dstring = dstring + f" {k[0]}x{k[1]}"
        print(f"  Sizes:{dstring}")
        if start is None:
            print("  First Time: No Valid Timestamps")
            print("  Last Time: No Valid Timestamps")
            print("  Duration: Unknown")
        else:
            print(f"  First Time: {start * ns_to_sec} ({_nanos_to_string(start)})")
            print(f"  Last Time: {end * ns_to_sec} ({_nanos_to_string(end)})")
            print(f"  Duration: {(end - start) * ns_to_sec} seconds")
        print(f"  Incomplete Scans: {incomplete_count}")
        if verbose:
            for i in incomplete_scans:
                print(i)

    atexit.register(exit_handler)


@click.command
@click.argument('beams', required=True,
              type=click.Choice(['8', '16', '32', '64', '128']))
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_reduce(ctx: SourceCommandContext, beams: str, **kwargs) -> None:
    """
    reduce the number of beams for each source to the specified beam count
    """
    # validate input
    for i, m in enumerate(ctx.scan_source.metadata):
        if int(beams) > m.format.columns_per_frame:
            raise click.exceptions.UsageError(
                f"selected beams count can't be larger than input, source[{i}] has"
                f" a beam count of {m.format.pixels_per_column}, but {beams} selected")


@click.command
@click.argument("image_path", required=True,
              type=click.Path(exists=True, dir_okay=False))
@click.option('--fields', default=None, type=str,
              help="Comma separated list of field names to narrow down which fields "
                   " the mask will be applied to.")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR)
def source_mask(ctx: SourceCommandContext, image_path: str, fields: str, **kwargs) -> None:
    """
    Applies a 2D mask to all streamed lidar scans
    """
    from PIL import Image
    import ouster.sdk.client.scan_ops as so
    from ouster.sdk.client.data import destagger

    image = Image.open(image_path)

    if image.mode != 'L':
        click.secho(f"image [{image_path}] is not an 8-bit grayscale,"
                    " performing conversion", fg="yellow")
        image = image.convert('L')

    # validate input
    masks = []
    for i, m in enumerate(ctx.scan_source.metadata):
        H, W = m.format.pixels_per_column, m.format.columns_per_frame
        image_cpy = image
        if image.height != H or image.width != W:
            click.secho(f"mask image doesn't match the size ({W}, {H}) "
                        f"for sensor[{i}], will scale", fg="yellow")
            image_cpy = image.resize((W, H))

        # mask should always be in the range value of {0, 1} the mask operator is applied
        # to the raw scans, so we need to stagger the input image correctly per input sensor
        mask = np.array(np.array(image_cpy) / 255.0)
        mask = destagger(m, mask, inverse=True)
        masks.append(mask)

    field_list = None if fields is None else fields.strip().split(',')
    scan_iter = ctx.scan_iter

    def mask_iter():
        for scans in scan_iter:
            for idx, scan in enumerate(scans):
                if scan:
                    so.mask(scan, field_list, masks[idx])
            yield scans

    ctx.scan_iter = mask_iter()


class SourceMultiCommand(click.MultiCommand):
    """This class implements the ouster-cli source command group.  It uses the
    `io_type` method to determine the source type and map it to the
    available sub commands for that type.

    The source is also added to the click context so that sub commands that use
    @click.pass_context have access to it."""

    def __init__(self, *args, **kwargs):
        kwargs['no_args_is_help'] = True

        super().__init__(*args, **kwargs)
        self.commands = {
            'ANY': {
                'viz': source_viz,
                'slice': source_slice,
                'clip': source_clip,
                'reduce': source_reduce,
                'mask': source_mask,
                'stats': source_stats
            },
            OusterIoType.SENSOR: {
                'config': sensor_cli.sensor_config,
                'metadata': sensor_cli.sensor_metadata,
                'save': SourceSaveCommand('save', context_settings=dict(ignore_unknown_options=True,
                                                                        allow_extra_args=True)),
                'userdata': sensor_cli.sensor_userdata,
                'save_raw': source_save_raw,
                'plumb': source_plumb,
                'network': sensor_cli.sensor_network,
                'diagnostics': sensor_cli.sensor_diagnostics,
            },
            OusterIoType.PCAP: {
                'info': pcap_cli.pcap_info,
                'save': SourceSaveCommand('save', context_settings=dict(ignore_unknown_options=True,
                                                                        allow_extra_args=True)),
                'save_raw': source_save_raw,
                'plumb': source_plumb,
            },
            OusterIoType.OSF: {
                'dump': osf_cli.osf_dump,
                'info': osf_cli.osf_info,
                'metadata': osf_cli.osf_metadata,
                'parse': osf_cli.osf_parse,
                'save': SourceSaveCommand('save', context_settings=dict(ignore_unknown_options=True,
                                                                        allow_extra_args=True)),
            },
            OusterIoType.BAG: {
                'info': bag_info,
                'metadata': bag_metadata,
                'save_raw': source_save_raw,
                'save': SourceSaveCommand('save', context_settings=dict(ignore_unknown_options=True,
                                                                        allow_extra_args=True)),
            }
        }

    def get_supported_source_types(self):
        return [iotype for iotype in self.commands.keys() if isinstance(iotype, OusterIoType)]

    def get_source_file_extension_str(self):
        exts = sorted(
            [extension_from_io_type(src_type)
                for src_type in self.commands.keys() if extension_from_io_type(src_type)]
        )
        return _join_with_conjunction(exts)

    def list_commands(self, click_ctx: click.core.Context):
        """Get the source type from the click context
        and return the list of appropriate sub command names"""
        source = click_ctx.params.get(_source_arg_name)
        source = source.split(',')[0] if source else None

        if not source and CliArgs().has_any_of(click_ctx.help_option_names):
            # Build a map from command name to command
            command_to_types = {}
            for src_type in self.commands.keys():
                for command_name in self.commands[src_type].keys():
                    if command_name not in command_to_types:
                        command_to_types[command_name] = {}
                    if src_type == "ANY":
                        for supported_src_type in self.get_supported_source_types():
                            command_to_types[command_name][supported_src_type] = self.commands[src_type][command_name]
                    else:
                        command_to_types[command_name][src_type] = self.commands[src_type][command_name]

            # Prefix command name with names of supported source types
            command_to_types_renamed = {}
            for key, value in command_to_types.items():
                prefix = _join_with_conjunction(
                    [t.name.upper() for t in value.keys()], separator="|", conjunction="")
                command_to_types_renamed[f"{prefix} {key}"] = value

            return command_to_types_renamed

        file_extensions_str = self.get_source_file_extension_str()
        if not source:
            param_decls = [_source_arg_name]
            param = click.core.Argument(param_decls=param_decls)
            raise click.exceptions.MissingParameter(
                None, click_ctx, param=param)
        try:
            return {**self.commands[io_type(source)], **self.commands["ANY"]}
        except ValueError as e:  # noqa: F841
            click.echo(click_ctx.get_usage())
            raise click.exceptions.UsageError("Source type expected to be a sensor hostname, "
                                              f"ip address, or a(n) {file_extensions_str} file. "
                                              "For a sensor source, please check that you can "
                                              "ping the sensor hostname/ip address. For a file "
                                              "source, please check that the file path you have "
                                              "provided exists.")
        except KeyError as e:  # noqa: F841
            click.echo(click_ctx.get_usage())
            raise click.exceptions.UsageError("Source type expected to be a sensor hostname, "
                                              f"ip address, or a(n) {file_extensions_str} file. "
                                              "For a sensor source, please check that you can "
                                              "ping the sensor hostname/ip address. For a file "
                                              "source, please check that the file path you have "
                                              "provided exists.")

    def get_command(self, click_ctx: click.core.Context, name: str):
        """Get the click.Command object for the given command name"""
        source = click_ctx.params.get(_source_arg_name)
        click_ctx.ensure_object(SourceCommandContext)
        ctx: SourceCommandContext = click_ctx.obj
        # add source to context so the command can access it
        ctx.source_uri = source

        command_list = self.list_commands(click_ctx)
        if name in command_list:
            if not source:
                # If called by --help (without source), return the first implementation of a command
                # NOTE: This results in help printing the docstring of only the first implementation
                return command_list[name][list(command_list[name].keys())[0]]
            else:
                ctx.invoked_command_names.append(name)
                return command_list[name]
        return None

    def invoke(self, click_ctx: click.core.Context):
        """Called when the source command is invoked.
        If called without any args, prints the help.
        Otherwise, the superclass method is called."""
        if not click_ctx.protected_args:
            print(self.get_help(click_ctx))
            return
        super().invoke(click_ctx)


@cli.group(cls=SourceMultiCommand, chain=True)
@click.argument(_source_arg_name, required=True)
# TODO[UN]: should we implement this as options instead of flag similar to `on_eof`?
@click.option('--loop', is_flag=True, default=False, hidden=True,
              help="Restart from begining when the end of the file is reached")
@click.option('-m', '--meta', required=False, type=click_ro_file, multiple=True,
              help="Metadata for PCAP or BAG, helpful if automatic metadata resolution"
                   " fails or no metadata is present in the bag file.")
@click.option('-l', '--lidar-port', default=None, type=int, help="Dest. port of lidar data")
@click.option('-i', '--imu-port', default=None, type=int, help="Dest. port of imu data")
@click.option('-x', '--do-not-reinitialize', is_flag=True, default=False,
              help="Do not reinitialize (by default it will reinitialize if needed)")
@click.option('-y', '--no-auto-udp-dest', is_flag=True, default=False,
              help="Do not automatically set udp_dest (by default it will auto set udp_dest")
@click.option('-s', '--soft-id-check', is_flag=True,
              help="Continue parsing lidar packets even if init_id/sn doesn't match with metadata")  # noqa
@click.option('-t', '--timeout', default=1.0, help="Seconds to wait for data", show_default=True)
@click.option('-F', '--filter', is_flag=True, help="Drop scans missing data")
@click.option('-e', '--extrinsics', type=str, required=False,
              help="Use this arg to adjust Lidar sensor extrinsics of the source."
                    "\nSupported formats:"
                    "\n\n-e extrinsics.json ; A json file containing a per sensor extrinsics"
                    "\n\n-e identity ; Use this to override any stored extrinsics with identity matrix"
                    "\n\n-e X,Y,Z,R,P,Y ; 'X Y Z' for position (meters), 'R P Y' represent euler angles (deg)"
                    "\n\n-e X,Y,Z,QX,QY,QZ,QW ; 'X Y Z' for position (meters), 'QX, QY QZ, QW' represent a quaternion"
                    "\n\n-e n1,n2,..,n16 ; 16 float representing a 2D array in a row-major order"
                    "\n\nIf more than one sensor is present in the source and this argument is used"
                    " then the same extrinsics will be applied to all sensors except when using"
                    " an extrinsics file.")
@click.option('-p', '--initial-pose', type=str, required=False,
              help="Use this arg to set the starting pose of the source relative to the map origin when using "
                    "localization.\nSupported formats:"
                    "\n\n-e X,Y,Z,R,P,Y ; 'X Y Z' for position (meters), 'R P Y' represent euler angles (deg)"
                    "\n\n-e X,Y,Z,QX,QY,QZ,QW ; 'X Y Z' for position (meters), 'QX, QY QZ, QW' represent a quaternion"
                    "\n\n-e n1,n2,..,n16 ; 16 float representing a 2D array in a row-major order")
@click.option('--fields', default=None, type=str,
              help="Comma separated list of field names to retrieve from the source."
                   " If not specified all (OSF) or defaults based on the packet format"
                   " (PCAP, Sensor, BAG) are loaded.")
def source(source, loop: bool, meta: Tuple[str, ...],
           lidar_port: int, imu_port: int,
           extrinsics: Optional[str], initial_pose: Optional[str],
           do_not_reinitialize: bool, no_auto_udp_dest: bool,
           soft_id_check: bool, timeout: int, filter: bool, fields: Optional[str]):
    """Run a command with the specified source (SENSOR, PCAP, BAG, or OSF) as SOURCE.
    For example, a sensor source: ouster-cli source os1-992xxx.local viz.
    To connect to multiple sensors you can provide them as a comma separated list.
    For example: ouster-cli source 192.168.1.201,192.168.1.202 viz.
    """
    # old ouster-mapping package may interfere the ouster-sdk package
    is_ouster_mapping_installed()


@source.result_callback()
@click.pass_context
def process_commands(click_ctx: click.core.Context, callbacks: Iterable[SourceCommandCallback],
                     source: str, loop: bool,
                     meta: Optional[Tuple[str, ...]], lidar_port: int, imu_port: int,
                     extrinsics: Optional[str], initial_pose: Optional[str],
                     do_not_reinitialize: bool, no_auto_udp_dest: bool,
                     soft_id_check: bool, timeout: int, filter: bool, fields: Optional[str]) -> None:
    """Process all commands in a SourceMultiCommand, using each command's callback"""

    callbacks = list(callbacks)
    ctx: SourceCommandContext = click_ctx.obj
    command_names = ctx.invoked_command_names
    resolved_extrinsics: Optional[Union[str, np.ndarray]] = None
    resolved_initial_pose: Optional[np.ndarray] = None

    if extrinsics:
        resolved_extrinsics = parse_extrinsics_from_string(extrinsics)

    if initial_pose:
        resolved_initial_pose = parse_extrinsics_from_string(initial_pose)
        if (not isinstance(resolved_initial_pose, np.ndarray) or
                resolved_initial_pose.shape != (4, 4)):
            raise click.exceptions.UsageError(f"'error processing initial_pose: {initial_pose},"
                                              " check help for proper usage")

    if not meta:
        meta = None

    # save some options for use later
    ctx.source_options["meta"] = meta
    ctx.source_options["do_not_reinitialize"] = do_not_reinitialize
    ctx.source_options["no_auto_udp_dest"] = no_auto_udp_dest
    ctx.source_options["soft_id_check"] = soft_id_check
    ctx.source_options["lidar_port"] = lidar_port
    ctx.source_options["imu_port"] = imu_port
    ctx.source_options["timeout"] = timeout
    ctx.source_options["resolved_extrinsics"] = resolved_extrinsics

    # ---- Lint commands ----
    # Ensure that no commands are duplicated unless command type is PROCESSOR
    names_duplicate_check = set()
    for idx, name in enumerate(command_names):
        if callbacks[idx].type == SourceCommandType.PROCESSOR:
            continue
        if name in names_duplicate_check:
            raise click.exceptions.UsageError(f"'{name}' is duplicated in the multi-command chain. "
                                              "Please invoke it only once. ")
        names_duplicate_check.add(name)

    # Ensure that no other commands are present, if a MULTICOMMAND_UNSUPPORTED
    # is present
    multicommand = True
    for idx, c in enumerate(callbacks):
        if c.type is SourceCommandType.MULTICOMMAND_UNSUPPORTED:
            multicommand = False
            if len(callbacks) != 1:
                raise click.exceptions.UsageError(f"'{command_names[idx]}' does not support multi-command chaining. "
                                                  "Please invoke it without other commands. ")

    # Ensure that a consumer is always last
    last_consumer_name, last_consumer_idx = None, None
    last_processor_name, last_processor_idx = None, None
    for idx, c in enumerate(callbacks):
        if c.type in [SourceCommandType.PROCESSOR_UNREPEATABLE, SourceCommandType.PROCESSOR]:
            last_processor_idx = idx
            last_processor_name = command_names[idx]
        elif c.type is SourceCommandType.CONSUMER:
            last_consumer_idx = idx
            last_consumer_name = command_names[idx]

    if multicommand:
        if last_consumer_idx is None:
            raise click.exceptions.UsageError("Must have a consumer such as 'viz' or 'save'.")
        if (last_processor_idx is not None) and (last_processor_idx > last_consumer_idx):
            raise click.exceptions.UsageError(f"'{last_processor_name}' must be invoked before "
                                              f"'{last_consumer_name}'. Please reorder the multi-command chain. ")

    if not multicommand:
        # Execute single non-multicommand command
        c = callbacks[0]
        c.callback_fn(ctx)
    else:
        # Execute multicommands
        # Open source

        field_names = None
        if fields is not None:
            field_names = fields.strip().split(',')

        source_list = [url.strip() for url in source.split(',') if url.strip()]

        try:
            source = open_source(source_list,
                                 sensor_idx=-1,
                                 extrinsics=resolved_extrinsics,
                                 meta=meta,
                                 cycle=loop,
                                 lidar_port=lidar_port, imu_port=imu_port,
                                 do_not_reinitialize=do_not_reinitialize,
                                 no_auto_udp_dest=no_auto_udp_dest,
                                 soft_id_check=soft_id_check,
                                 timeout=timeout, complete=filter,
                                 field_names=field_names)

            if resolved_initial_pose is not None:
                for meta in source.metadata:
                    meta.extrinsic = meta.extrinsic @ resolved_initial_pose

            # HACK: We need to redesign how the ScanSource is passed is passed and
            # used, the current pipeline skips the scan_source and simply uses the
            # iterators which then results in this problem where any operation that
            # may affect the scan_source isn't taking effect such as the reduce
            if command_names[0] == "reduce":
                beams_idx = sys.argv.index("reduce")
                beams = [int(sys.argv[beams_idx + 1])] * source.sensors_count
                from ouster.sdk.client import MultiReducedScanSource
                ctx.scan_source = MultiReducedScanSource(
                    source, beams=beams)
            else:
                if "reduce" in command_names[1:]:
                    click.secho("ERROR: reduce needs to be the first command after source.",
                                fg="red")
                    return
                ctx.scan_source = source

        except SourceURLException as e:
            sub_exception = e.get_sub_exception()
            is_dupe_port = isinstance(sub_exception, PcapDuplicatePortException)
            if is_dupe_port and soft_id_check:
                click.secho("ERROR: --soft-id-check is not supported for multi-sensor datasets.",
                            fg="red")
                return
            else:
                raise

        if ctx.scan_source.is_indexed and len(ctx.scan_source) == 0:
            click.secho("WARNING: Source contains no scans.", fg="yellow")

        # print any timeout exceptions we get and lazily instantiate the scan
        # source iterator so consumers have a chance to set flags like loop

        def catch_iter():
            try:
                last_dropped = 0
                for scan in iter(ctx.scan_source):
                    if hasattr(ctx.scan_source, "dropped_scans"):
                        dropped = ctx.scan_source.dropped_scans
                        if dropped > last_dropped:
                            click.echo(click.style(f"Warning: Dropped {dropped - last_dropped} lidar scans.",
                                                   fg="yellow"))
                            last_dropped = dropped
                    yield scan
            except ClientTimeout as ex:
                click.echo(click.style(f"Error: {ex}", fg="red"))
            except FileExistsError as ex:
                click.echo(click.style(f"Error: {ex}. Add --overwrite flag to overwrite and continue anyways.",
                                       fg="red"))
            return
        ctx.scan_iter = catch_iter()

        # speed up slicing on indexed OSF if slicing comes first
        global _last_slice
        if command_names[0] == "slice" and _last_slice:
            # we can only speed it up if its indexed and OSF at the moment
            if isinstance(ctx.scan_source, OsfScanSource) and ctx.scan_source.is_indexed:
                # at the moment we can only handle index based slices (where the start is not a float)
                # TODO: support time based slices
                if type(_last_slice[0]) is not float and type(_last_slice[1]) is not float:
                    # finally calculate wrap-around start and end indexes assuming cycle is set
                    # TODO: revist when we revist cycle/loop
                    start_index = _last_slice[0]
                    end_index = None
                    if _last_slice[1] is not None:
                        end_index = _last_slice[1] - start_index
                    # slice the source to jump to our desired start point quickly
                    ctx.scan_source = ctx.scan_source.slice(slice(start_index, None, None))
                    callbacks = callbacks[1:]  # remove the callback since we dont need it now
                    # finally apply the rest of the slice (end and interval)
                    ctx.scan_iter = islice(ctx.scan_iter, 0, end_index, _last_slice[2])
            _last_slice = None  # globals are the root of all evil

        try:
            # Execute multicommand callbacks

            # Dataflow between callbacks occur entirely through the scan iterator. Callbacks may additionally
            # register functions in ctx.thread_fns to be run in individual threads
            # Callback invariants:
            #   1. Must leave ctx.scan_iter in a consumable state after invocation.
            #       If a callback consumes the original ctx.scan_iter, it must set ctx.scan_iter
            #       to an unconsumed iterator (via CoupledTee or a new iterator)
            #   2. May add a Callable[None -> None] to ctx.thread_fns. These will be executed in
            #       individual threads.
            #   3. A single callback may also register a function to be run from the main thread in
            #       ctx.main_thread_fn. This is required to support viz on macOS, where openGL
            #       applications must be run from the main thread
            #   4. Any registered thread_fn must support termination of execution via
            #       ctx.terminate_evt.is_set()
            #   5. Every registered thread_fn must request the ouster-cli process terminate by calling
            #       ctx.terminate_evt.set() before terminating

            # Most callbacks will do one of the following:
            #   1. execute immediately, and take no further action (ie, config)
            #   2. map a processing function onto scans, such that processing is implicitly called while iterating
            #   3. set ctx.scan_iter to a new iterator, and register a processing thread in ctx.thread_fns
            #   4. create a CoupledTee from ctx.scan_iter, and re-set ctx.scan_iter to one of the resultant tees

            ctx.thread_fns = []
            ctx.main_thread_fn = None
            ctx.terminate_evt = threading.Event()
            for c in callbacks:
                c.callback_fn(ctx)

            # Create threads from functions registered by the callbacks
            threads = []
            for thread_fn in ctx.thread_fns:
                threads.append(threading.Thread(target=thread_fn))

            # Define a function to consume ctx.scan_iter
            def pipeline_flush():
                try:
                    for _ in ctx.scan_iter:
                        pass
                except Exception as ex:
                    # Terminate everything if we get an unhandled exception
                    ctx.terminate_evt.set()
                    raise ex

            threads.append(threading.Thread(target=pipeline_flush))

            # Start all threads
            for thread in threads:
                thread.start()

            # Execute main thread fn, if set
            if ctx.main_thread_fn is not None:
                ctx.main_thread_fn()
                ctx.terminate_evt.set()

            # Wait for threads to terminate
            while len(threads) > 0:
                # On Windows thread.join(None) blocks through KeyboardInterrupts
                # Make sure that we only block temporarily so we can catch them
                for thread in threads:
                    thread.join(0.1)
                    if not thread.is_alive():
                        threads.remove(thread)
                        continue

            # TODO: https://ouster.atlassian.net/browse/FLEETSW-6470
            if "id_error_count" in dir(ctx.scan_source) and ctx.scan_source.id_error_count > 0:
                print(f"WARNING: {ctx.scan_source.id_error_count} lidar_packets with "
                      f"mismatched init_id/sn were detected.")
                if not soft_id_check:
                    print("NOTE: To disable strict init_id/sn checking use "
                          "--soft-id-check option (may lead to parsing "
                          "errors)")
            if "size_error_count" in dir(ctx.scan_source) and ctx.scan_source.size_error_count > 0:
                print(f"WARNING: {ctx.scan_source.size_error_count} lidar_packets with unexpected"
                      f" size detected and discarded. You may have the incorrect udp_profile_lidar in your metadata.")
        except KeyboardInterrupt:
            print("Termination requested, shutting down...")
            ctx.terminate_evt.set()
        finally:
            try:
                ctx.scan_source.close()
            except:  # noqa: E722
                pass
