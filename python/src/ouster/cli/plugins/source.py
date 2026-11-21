from typing import Optional, Iterable, List, Tuple, Union, MutableMapping, Any, Dict, cast
import atexit
import click
import re
import sys
import importlib.metadata
import threading
import copy
import numpy as np
import glob as libglob
import operator
import os
from itertools import islice
from functools import partial
from ouster.cli.core import cli
from ouster.cli.core.cli_args import CliArgs
from ouster.cli.core.util import click_ro_file
from ouster.sdk import (open_source, SourceURLException,
                        core, sensor)
from ouster.sdk.algorithm import align_clouds
from ouster.sdk.core import (collate, FrameSetSource, MultiFrameSetSource, FrameSet)
from ouster.sdk.sensor import ClientTimeout
from ouster.sdk.core import (extension_from_io_type, io_type, OusterIoType,
                             ChanField, FieldClass, LidarFrame, XYZLut, dewarp,
                             destagger, stagger, CameraFrame,
                             get_rot_matrix_to_align_to_gravity, ZoneSetOutputFilter)
from ouster.sdk.algorithm import normals, WallSegEngine, GroundSegEngine, plumb
from ouster.sdk.pcap import PcapDuplicatePortException
import ouster.cli.plugins.source_pcap as pcap_cli
import ouster.cli.plugins.source_osf as osf_cli
from ouster.sdk.util.extrinsics import parse_extrinsics_from_string
from ouster.sdk.util import ProgressBar
from .source_save import (SourceSaveCommand, source_save_raw)
from .source_util import (CoupledTee,
                          SourceCommandContext,
                          SourceCommandCallback,
                          SourceCommandType,
                          source_multicommand,
                          _join_with_conjunction,
                          _nanos_to_string,
                          sensor_frame_xyzlut)
import ouster.sdk.mapping as mapping
import ouster.sdk.viz as viz
from .source_po_viz import PoseOptimizerViz
from .source_bag import bag_info

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


_viz_wants_cycle = False


def parse_resolution(click_ctx: Optional[click.core.Context],
                     param: Optional[click.core.Argument], value: str):
    if value is None:
        return None

    # Convert to lowercase to allow "1080P", "4K", etc.
    val_lower = value.lower().strip()

    # Handle named resolutions
    if val_lower == "144p":
        return (256, 144)
    elif val_lower == "240p":
        return (426, 240)
    elif val_lower == "360p":
        return (640, 360)
    elif val_lower == "480p":
        return (854, 480)
    elif val_lower == "720p":
        return (1280, 720)
    elif val_lower == "1080p":
        return (1920, 1080)
    elif val_lower == "2k":
        return (2560, 1440)
    elif val_lower == "4k":
        return (3840, 2160)
    elif val_lower == "8k":
        return (7680, 4320)

    # Otherwise, try with <width>x<height> pattern
    match_size = re.match(r'^(\d+)x(\d+)$', val_lower)
    if match_size:
        width, height = map(int, match_size.groups())
        # Check for invalid (zero or negative) dimensions
        if width <= 0 or height <= 0:
            raise click.exceptions.BadParameter(
                f"Invalid resolution '{value}': width and height must be > 0."
            )
        return (width, height)

    match_scale = re.match(r'^(\d+(\.\d+)?)x$', val_lower)
    if match_scale:
        scale = float(match_scale.group(1))
        if scale <= 0:
            raise click.exceptions.BadParameter(
                f"Invalid scale factor '{value}': scale must be > 0."
            )
        return scale

    # If it doesn't match any known pattern, raise an error
    raise click.exceptions.BadParameter(
        f"Invalid resolution '{value}'. Please use '<width>x<height>' or '1080p', '2k', '4k', '8k'."
    )


def _hex_to_color(value: str) -> Tuple[float, float, float, float]:
    if len(value) == 3:
        real_value = "".join(2 * s for s in value)
    elif len(value) == 6:
        real_value = value
    else:
        raise click.exceptions.BadParameter(f"'{value}' is not a valid code. Must have 3 or 6 digits.")
    try:
        int_value = int(real_value, 16)
    except ValueError:
        raise click.exceptions.BadParameter(f"'{value}' is not a valid hex code.")
    r = int_value >> 16
    g = int_value >> 8 & 0xFF
    b = int_value & 0xFF
    return (r / 255.0, g / 255.0, b / 255.0, 1.0)


def parse_color(click_ctx: Optional[click.core.Context],
                param: Optional[click.core.Argument], value: str):
    if len(value) > 1 and value[0] == "0" and value[1] == "x":
        return _hex_to_color(value[2:])
    if value[0] == "#":
        return _hex_to_color(value[1:])

    value = value.lower()
    colors: Dict[str, Tuple[float, float, float, float]] = {}
    colors["white"] = (1.0, 1.0, 1.0, 1.0)
    colors["black"] = (0.0, 0.0, 0.0, 1.0)
    colors["gray"] = (0.5, 0.5, 0.5, 1.0)
    colors["red"] = (1.0, 0.0, 0.0, 1.0)
    colors["green"] = (0.0, 1.0, 0.0, 1.0)
    colors["blue"] = (0.0, 0.0, 1.0, 1.0)
    colors["ecru"] = (194 / 255, 178 / 255, 128 / 255, 1.0)
    if value not in colors:
        possibilities = list(colors.keys())
        possibilities.sort()
        raise click.exceptions.BadParameter(f"Color '{value}' not found by name."
                                            f" Try one of: {', '.join(possibilities)}.")
    return colors[value]


def parse_resolution_no_scale(click_ctx: Optional[click.core.Context],
                              param: Optional[click.core.Argument], value: str):
    result = parse_resolution(click_ctx, param, value)
    if isinstance(result, float):
        raise click.exceptions.BadParameter(
            "Scale factors are not supported for --resolution. "
            "Use '<width>x<height>' (e.g. 1920x1080) or a known name (e.g. 1080p, 2k, 4k, 8k)."
        )
    return result


def parse_gps_constraints_weights(
        gps_constraints_weights: Optional[Union[str, Tuple[float, float, float]]] = None
) -> np.ndarray:
    """Parse GPS constraint weights from CLI input."""
    if gps_constraints_weights is None:
        return np.array([0.01, 0.01, 0.001], dtype=float)

    if isinstance(gps_constraints_weights, tuple):
        if len(gps_constraints_weights) != 3:
            raise ValueError(
                "Expected gps_constraints_weights to be a 3-tuple (WX, WY, WZ)")
        weights = np.array([float(w) for w in gps_constraints_weights],
                           dtype=float)
    elif isinstance(gps_constraints_weights, str):
        parts = re.split(r"[,\s]+", gps_constraints_weights.strip())
        if len(parts) != 3:
            raise ValueError(
                "Expected 'WX,WY,WZ' (3 numbers), e.g. '0.01,0.01,0.001'")
        weights = np.array([float(p) for p in parts], dtype=float)
    else:
        raise TypeError(
            "gps_constraints_weights must be a 3-tuple or a 'WX,WY,WZ' string")

    return weights


@click.command()
@click.option("--fullscreen", is_flag=True, help="Put the viz into fullscreen mode.")
@click.option("-p", "--pause", is_flag=True, help="Pause at first lidar frame")
@click.option("-e", "--on-eof", default='loop', type=click.Choice(['loop', 'stop', 'exit']),
              help="Loop, stop or exit after reaching end of file")
@click.option('-r',
              '--rate',
              default="1",
              help="Playback rate.",
              type=click.Choice(["0.1", "0.25", "0.5", "0.75", "1", "1.5", "2", "3", "max"]))
@click.option("--pause-at",
              default=-1,
              help="Lidar frame number to pause at")
@click.option("--accum-num",
              default=None,
              type=int,
              help="Accumulate up to this number of past frames for visualization. "
                   "Use <= 0 for unlimited. Defaults to 100 if '--accum-every' or '--accum-every-m' is set.")
@click.option("--accum-every",
              default=None,
              type=int,
              help="Add a new frame to the accumulator every this many frames.")
@click.option("--accum-every-m",
              default=None,
              type=float,
              help="Add a new frame to the accumulator after this many meters of travel.")
@click.option("--map", "_map",
              is_flag=True,
              help="If set, add random points from every frame into an overall map for visualization. "
                   "Enabled if either '--map-ratio' or '--map-size' are set.")
@click.option("--map-ratio",
              default=None, type=float,
              help="Fraction of random points in every frame to add to overall map (0, 1]. [default: 0.01]")
@click.option("--map-size",
              default=None, type=int,
              help="Max number of points in overall map before discarding. [default: 1500000]")
@click.option("--global-map", default=None, type=str,
              help="A path to a ply file that represents the global map to display in the ouster-viz. "
                   "When using this option with the 'localize' command it will replace the visualized global"
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
              help="Set the point size of the global map")
@click.option("--global-map-color", default=True, type=bool, show_default=True,
              help="Enable or disable coloring of the global map (if available)")
@click.option("--imu-plot-options", default='only_gyro',
              type=click.Choice(['only_gyro', 'only_acc', 'both', 'none']),
              help="Specifies which fields to display in IMU plots.")
@click.option("-m", "--maximize", type=bool, is_flag=True, help="Maximize the window")
@click.option("--resolution", default=None, callback=parse_resolution_no_scale,
              help="Specify the window resolution as <width>x<height> (e.g. 1920x1080) "
              "or use a known name (e.g. 1080p, 2k, 4k, 8k). "
              "If the requested resolution exceeds the available screen area, "
              "the OS will constrain the window to fit within the screen. "
              "Ignored if '--maximize' or '--fullscreen' is set.")
@click.option("--screenshot-resolution", default=None, callback=parse_resolution,
              help="Specify a custom resolution as <width>x<height> (e.g. 1920x1080), a scale factor <scale_factor>x"
              " (e.g. 2x, 1.5x) or use a known name (e.g. 1080p, 2k, 4k, 8k).")
@click.option("--clear-on-loop", type=bool, is_flag=True, help="Clear history and trajectories on loop.")
@click.option("--subframes", type=click.IntRange(min=0), default=0, help="Number of subframes to visualize per frame. "
              "If > 0, the viz will interpolate linearly between frames to create and visualize this many subframes.")
@click.option("--title", type=str, default=None, help="Title of viz window.")
@click.option("--save", type=str, default=None, help="Save the viz config to the specified file on exit.")
@click.option("--load", type=click.Path(exists=True), default=None,
              help="Load the viz config from the specified file on start."
                   " Can be set via OUSTER_SDK_VIZ_CONFIG env var.",
              envvar="OUSTER_SDK_VIZ_CONFIG")
@click.option("-b", "--background", default="black", show_default=True, callback=parse_color,
              help="Background color. Accepts hex codes or simple color names."
                   " (e.g. 0x777777, #888 or red)")
@click.option("--camera-default-range", default=viz.DEFAULT_CAMERA_RANGE_MM, show_default=True,
              type=click.IntRange(min=0),
              help="Distance (mm) at which to draw the pixels of camera frames that have no RANGE field, "
                   "so their images show as a 3D point cloud, measured as forward distance (a flat image "
                   "plane). 0 draws no cloud for them.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_viz(ctx: SourceCommandContext,
               pause: bool,
               on_eof: str,
               pause_at: int,
               accum_num: Optional[int],
               accum_every: Optional[int],
               accum_every_m: Optional[float],
               _map: bool,
               map_ratio: float,
               rate: str,
               map_size: int,
               global_map: Optional[str],
               global_map_min_z: Optional[float],
               global_map_max_z: Optional[float],
               global_map_flatten: bool,
               global_map_voxel_size: Optional[float],
               global_map_point_size: float,
               global_map_color: bool,
               imu_plot_options: str,
               resolution: Optional[str],
               screenshot_resolution: Optional[str],
               maximize: bool,
               fullscreen: bool,
               clear_on_loop: bool, subframes: int,
               title: Optional[str],
               save: Optional[str],
               load: Optional[str],
               background: Tuple[float, float, float, float],
               camera_default_range: int) -> None:
    """Visualize data in a 3D viewer."""
    try:
        from ouster.sdk.viz import SimpleViz, ImuVisualizationConfig, Cloud
        from ouster.cli.plugins.source_slam_viz import SlamLocalMapViz
    except ImportError as e:
        raise click.ClickException(str(e))

    source = ctx.frame_set_source
    assert source is not None

    # ugly workarounds ensue
    if on_eof == 'loop':
        global _viz_wants_cycle
        _viz_wants_cycle = True
        # don't pass loop to the viz as we want the CLI to perform the looping
        on_eof = 'exit'

    if pause and pause_at == -1:
        pause_at = 0

    # Determine how to set the rate
    viz_rate: Optional[float]
    if rate == "max":
        viz_rate = 0.0
    else:
        viz_rate = float(rate)
    if source.is_live:
        if viz_rate != 1.0:
            raise click.exceptions.UsageError("Can only set a rate of 1 for live sources")
        viz_rate = None

    ctx.frame_set_iter, tees = CoupledTee.tee(ctx.frame_set_iter,  # type: ignore
                                         terminate=ctx.terminate_evt,
                                         loop=_viz_wants_cycle)
    frames = tees[0]
    assert ctx.frame_set_source is not None
    metadata = ctx.frame_set_source.sensor_info

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
            rate=viz_rate, pause_at=pause_at, on_eof=on_eof,
            accum_max_num=accum_num,
            accum_min_dist_num=accum_every,
            accum_min_dist_meters=accum_every_m,
            map_enabled=_map,
            map_select_ratio=map_ratio,
            map_max_points=map_size,
            title=title or ("Ouster Viz: " + ctx.source_uri),
            imu_viz_config=ImuVisualizationConfig(options=imu_plot_options),
            maximized=maximize,
            fullscreen=fullscreen,
            resolution=resolution,
            screenshot_resolution=screenshot_resolution,
            clear_on_loop=clear_on_loop,
            subframes=subframes,
            background_color=background,
            camera_default_range_mm=camera_default_range
        )

        if "class_maps" in source.metadata_keys():
            print("Found class maps in metadata, adding to viz")
            class_maps = source.metadata("class_maps")
            if isinstance(class_maps, core.ClassMapSet):
                sv.set_class_maps(class_maps)

        map_path = ctx.get("localization.map", None) if global_map is None else global_map

        if map_path is not None:
            from ouster.cli.plugins.read_pointcloud_color import read_pointcloud_color
            from ouster.sdk.core import voxel_downsample, VoxelDownsampleStrategy
            click.echo("Start loading global points into VIZ")
            pts, rgb = read_pointcloud_color(map_path)
            if global_map_min_z:
                selected_pts = pts[:, 2] >= global_map_min_z
                pts = pts[selected_pts]
                rgb = rgb[selected_pts]
            if global_map_max_z:
                selected_pts = pts[:, 2] <= global_map_max_z
                pts = pts[selected_pts]
                rgb = rgb[selected_pts]
            if global_map_voxel_size:
                pts, [rgb] = voxel_downsample(pts, global_map_voxel_size,
                                              strategy=VoxelDownsampleStrategy.AVERAGE_POINT,
                                              attributes=[rgb])
            if global_map_flatten:
                pts[:, 2] = 0
            cloud_xyz = Cloud(len(pts))
            cloud_xyz.set_xyz(np.ascontiguousarray(pts.astype(np.float32)))
            if global_map_color:
                cloud_xyz.set_key(np.ascontiguousarray(rgb.astype(np.float32)))
            else:
                # use uniform default color
                cloud_xyz.set_key(np.full(len(pts), 1.0, np.float32))
            cloud_xyz.set_point_size(global_map_point_size)
            sv._viz.add(cloud_xyz)

        viz_frames = frames

        if ctx.slam_local_map_viz_enabled:
            viz_frames = SlamLocalMapViz(ctx, sv).wrap(viz_frames)

        if load:
            with open(load, "r") as f:
                sv._frame_viz.config.load(f.read())

        sv.run(viz_frames)
        ctx.terminate_evt.set()

        if save:
            with open(save, "w") as f:
                f.write(sv._frame_viz.config.save())
                f.write("\n")

    if ctx.main_thread_fn is not None:
        raise RuntimeError(
            "A main-thread required function has already been set.")
    ctx.main_thread_fn = viz_thread_fn  # type: ignore


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
    # to apply the appropriate cast
    type_caster = int if frame_based else float
    start = type_caster(start) if start is not None else start
    stop = type_caster(stop) if stop is not None else stop
    step = int(step) if step is not None else step
    # Store the argument so we can extract it for fast slicing if possible
    global _last_slice
    _last_slice = (start, stop, step)
    return start, stop, step, frame_based


def tslice(frames_iter, start, stop, step):
    start_time = None
    counter = 0
    for frame in frames_iter:
        frame_time = None
        for s in frame:
            if s:
                try:
                    frame_time = s.get_min_valid_receive_timestamp()
                except RuntimeError:
                    pass
                break
        if frame_time is None:
            click.secho("WARNING: Frame missing packet timestamps. "
                        "Yielding frame in time slice anyways.", fg='yellow')
            yield frame
            continue
        frame_time = frame_time / 1e9
        if start_time is None:
            start_time = frame_time
        dt = frame_time - start_time
        if dt >= start:
            if not stop or dt <= stop:
                if not step or counter % step == 0:
                    yield frame
                counter = counter + 1
            else:
                return


@click.command()
@click.argument('indices', required=True, callback=extract_slice_indices)
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_slice(ctx: SourceCommandContext,
                 indices: Tuple[Optional[int], Optional[int], Optional[int], bool]) -> None:
    """Slice LidarFrames streamed from SOURCE. Use the form [start]:[stop][:step].
    Optionally can specify start and stop as times relative to the start of the file
    using the units h (hours), min (minutes), s (seconds), or ms (milliseconds).
    For example: 10s:20s:2"""

    start, stop, step, frame_based = indices
    slice_method = islice if frame_based else tslice
    frames_iter = ctx.frame_set_iter

    def slice_iterator():
        for frame in slice_method(frames_iter(), start, stop, step):
            yield frame
    ctx.frame_set_iter = slice_iterator  # type: ignore


@click.command(name="normals")
@click.option("--sensor-coord", is_flag=True, default=False,
              help="Compute normals in the sensor coordinate")
@click.option("--pixel-search-range", type=int, default=1, show_default=True,
              help="Limit the neighbor pixel search to +/- this many pixels when estimating normals.")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR)
def source_normals(ctx: SourceCommandContext, sensor_coord: bool,
                   pixel_search_range: int) -> None:
    """Compute unit surface normals for each range-bearing frame.

    By default the point clouds are dewarped using per-column poses (when
    available) so normals are expressed in the global coordinate. Pass
    '--sensor-coord' to keep computations in the sensor coordinate instead.
    """
    if ctx.frame_set_iter is None or ctx.frame_set_source is None:
        raise click.ClickException("'normals' must be used after a source is initialized.")
    frame_set_iter = ctx.frame_set_iter
    # Key the LUT cache by the unified sensor index (the FrameSet frame-slot
    # index) so it stays aligned with the frame that produced it.
    sensor_infos = ctx.frame_set_source.sensor_info

    def normals_xyzlut(info: core.SensorInfo) -> XYZLut:
        # Body-frame XYZ when dewarping to world; sensor-frame with --sensor-coord.
        return sensor_frame_xyzlut(info) if sensor_coord else XYZLut(info)

    xyzluts: Dict[int, XYZLut] = {
        idx: normals_xyzlut(info) for idx, info in enumerate(sensor_infos)
    }

    def estimate_normals(frame: core.Frame, sensor_idx: int) -> None:
        if not frame.has_field(ChanField.RANGE):
            return
        info = _frame_extrinsic_info(frame) or frame.sensor_info
        xyzlut = xyzluts.get(sensor_idx)
        if xyzlut is None:
            xyzlut = normals_xyzlut(info)
            xyzluts[sensor_idx] = xyzlut
        h, w = frame.h, frame.w

        range_staggered = frame.field(ChanField.RANGE)
        xyz_staggered = xyzlut(range_staggered).reshape(h, w, 3)

        range_destaggered = destagger(info, range_staggered)
        xyz_destaggered = destagger(info, xyz_staggered)
        if not sensor_coord:
            poses = np.asarray(frame.body_to_world)
            xyz_destaggered = cast(np.ndarray, dewarp(xyz_destaggered, poses))
            origins = np.ascontiguousarray(
                (poses @ info.sensor_to_body)[:, :3, 3], dtype=np.float64)
            if origins.shape[0] == 1 and w > 1:
                sensor_origins_xyz = np.tile(origins, (w, 1))
            else:
                sensor_origins_xyz = origins
        else:
            sensor_origins_xyz = np.zeros((w, 3), dtype=np.float64)

        normals_destaggered: np.ndarray
        normals2_destaggered: Optional[np.ndarray] = None
        if frame.has_field(ChanField.RANGE2):
            range2_staggered = frame.field(ChanField.RANGE2)
            xyz2_staggered = xyzlut(range2_staggered).reshape(h, w, 3)

            range2_destaggered = destagger(info, range2_staggered)
            xyz2_destaggered = destagger(info, xyz2_staggered)
            if not sensor_coord:
                xyz2_destaggered = cast(np.ndarray, dewarp(xyz2_destaggered, poses))

            normals_destaggered, normals2_destaggered = normals(
                xyz_destaggered,
                range_destaggered,
                xyz2_destaggered,
                range2_destaggered,
                pixel_search_range=pixel_search_range,
                sensor_origins_xyz=sensor_origins_xyz,
            )
        else:
            normals_destaggered = normals(
                xyz_destaggered,
                range_destaggered,
                pixel_search_range=pixel_search_range,
                sensor_origins_xyz=sensor_origins_xyz,
            )

        normals_staggered = stagger(info, normals_destaggered).astype(np.float32, copy=False)

        def update_frame_field(field_name: str, values: np.ndarray) -> None:
            expected_shape = values.shape
            per_pixel_shape = expected_shape[2:]
            field_dtype = values.dtype.type
            if frame.has_field(field_name):
                field_view = frame.field(field_name)
                if field_view.shape != expected_shape or field_view.dtype != values.dtype:
                    frame.del_field(field_name)
                    field_view = frame.add_field(field_name, field_dtype, per_pixel_shape)
            else:
                field_view = frame.add_field(field_name, field_dtype, per_pixel_shape)
            field_view[:] = values

        update_frame_field("NORMALS", normals_staggered)

        if normals2_destaggered is not None:
            normals2_staggered = stagger(info, normals2_destaggered).astype(np.float32, copy=False)
            update_frame_field("NORMALS2", normals2_staggered)
        elif frame.has_field("NORMALS2"):
            frame.del_field("NORMALS2")

    def normals_iter() -> Iterable[FrameSet]:
        if frame_set_iter is None:
            return

        for frames in frame_set_iter():  # type: ignore[operator]
            out: List[Optional[core.Frame]] = []
            for idx, frame in enumerate(frames):
                if frame is None:
                    out.append(None)
                    continue

                try:
                    estimate_normals(frame, idx)
                except Exception as exc:
                    click.secho(f"Warning: failed to compute normals for frame {idx}: {exc}", fg='yellow')

                out.append(frame)
            yield frames if isinstance(frames, FrameSet) else FrameSet(out)

    ctx.frame_set_iter = normals_iter  # type: ignore[assignment]


@click.command(name="ground")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR)
def source_ground(ctx: SourceCommandContext) -> None:
    """Run ground segmentation and add a GROUND mask field.

    Accuracy is improved when frames already contain SLAM poses, or when the
    source was pre-plumbed with IMU data on firmware > 3.1 so gravity alignment
    is written into sensor extrinsics.
    """
    assert ctx.frame_set_iter is not None and ctx.frame_set_source is not None
    frame_set_iter = ctx.frame_set_iter
    ground = GroundSegEngine.create()

    def ground_iter() -> Iterable[FrameSet]:
        for frames in frame_set_iter():  # type: ignore[operator]
            try:
                ground.update(frames)
            except Exception as exc:
                click.secho(
                    f"Warning: failed to compute ground mask: {exc}",
                    fg='yellow',
                )
            yield frames

    ctx.frame_set_iter = ground_iter  # type: ignore[assignment]


@click.command(name="walls", hidden=True)
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR)
def source_walls(ctx: SourceCommandContext) -> None:
    """Run wall segmentation and add a WALL mask field."""
    assert ctx.frame_set_iter is not None and ctx.frame_set_source is not None
    frame_set_iter = ctx.frame_set_iter
    walls = WallSegEngine.create()

    def walls_iter() -> Iterable[FrameSet]:
        for frames in frame_set_iter():  # type: ignore[operator]
            try:
                walls.update(frames)
            except Exception as exc:
                click.secho(
                    f"Warning: failed to compute wall mask: {exc}",
                    fg='yellow',
                )
            yield frames

    ctx.frame_set_iter = walls_iter  # type: ignore[assignment]


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
    lower, upper = parsed_indices[0], parsed_indices[1]
    # Check that lower >= lower if both are provided
    if (lower is not None) and (upper is not None) and lower > upper:
        raise click.exceptions.BadParameter(
            "'upper' value must be greater or equal to 'lower'")
    return lower, upper


@click.command
@click.argument('fields', required=True, type=str)
@click.argument('indices', required=True, callback=extract_clip_indices)
@click.option('--out-of-range-value', default=0, show_default=True,
              help="The value used when replacing out of range values")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR)
def source_clip(ctx: SourceCommandContext, fields: str,
                indices: Tuple[Optional[int], Optional[int]], out_of_range_value: int,
                **kwargs) -> None:
    """
    Constrains the range of values of the specified fields to the range [lower, upper] inclusive.
    Any value beyond this range is replaced with the out-of-range-value (default is zero).
    Use the form clip FIELDS [lower[u]]:[upper[u]]; where 'FIELDS' is a comma separated list of field
    names (no spaces) that this operation will be applied to, the 'u' is an optional unit specifier;
    Supported units and their effects {mm: 1x, cm: 10x, dm: 100x, m: 1000x}.

    Usage example 1: 'clip RANGE,RANGE2 :50m' would zero any RANGE values of the frame higher than 50
    meters. If the metric unit is not supplied the passed value will be used with no change, that is if
    a user passes :50 then it would be evaluated as 50 millimeters for RANGE values and merely as 50 units
    for other fields.

    Usage example 2: 'clip RANGE 50m:50m' would only forward RANGE values of the frame that are
    exactly 50 meters.
    """
    import ouster.sdk.core.frame_ops as so

    frame_set_iter = ctx.frame_set_iter
    field_list = fields.strip().split(',')
    start, stop = indices
    real_start = start if start is not None else float('-inf')
    real_stop = stop if stop is not None else float('inf')

    def clip_iter():
        for frames in frame_set_iter():
            out = copy.deepcopy(frames)
            for frame in out:
                if frame:
                    so.clip(frame, field_list, real_start, real_stop, out_of_range_value)
            yield out

    ctx.frame_set_iter = clip_iter  # type: ignore


@click.command
@click.argument('axis_field', required=True, type=str)
@click.argument('indices', required=True, callback=extract_clip_indices)
@click.option('--filtered-fields', default=None, show_default=True,
              help="A comma separated list of field names to apply the filter to. "
                   "If not provided, the filter will be applied to all fields.")
@click.option('--invalid-value', default=0, type=float, show_default=True,
              help="The value to used for pixels that match the filter")
@click.option('--coord-frame', default="BODY", show_default=True,
              type=click.Choice(["SENSOR", "BODY", "WORLD"], case_sensitive=False),
              help="Coordinate frame for XYZ filtering (SENSOR=no extrinsics, "
                   "BODY=extrinsics, WORLD=dewarped)")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR)
def source_filter(ctx: SourceCommandContext, axis_field: str, indices: Tuple[Optional[int], Optional[int]],
                  filtered_fields: Optional[str], invalid_value: float,
                  coord_frame: str, **kwargs) -> None:
    """
    Apply a filter to frame data based on the specified axis/field and indices.

    This command processes lidar and camera frame data by filtering values along a specified
    axis (X, Y, Z) or an image coord (U, V) or a frame source field. The filter applies to all
    fields in the frame by default but the user can select a subset of field to apply the filter
    to. The filter works by updating values that match the condition to the invalid-value
    (default: 0). Camera frames are handled the same way as lidar frames, except they are not
    dewarped (a single per-frame pose is applied for WORLD-frame XYZ filtering).

    Usage:
    filter [--filtered-fields <fields>] [--invalid-value <value>]
           [--coord-frame <SENSOR|BODY|WORLD>] <axis_field> <indices>

    Examples:

        1) ouster-cli source <source_url> filter REFLECTIVITY 0:100 viz

        2) ouster-cli source <source_url> filter --filtered-fields REFLECTIVITY,SIGNAL X -10m:10m viz

        3) ouster-cli source <source_url> filter U :62 filter U 66: viz

    Notes:

        * It is possible to use the suffix 'mm', 'cm', 'dm', 'm' to specify the units of the indices. If
        no unit is provided the value is assumed to be in the field units.

        * when working with image coordinates (u, v) the indices are assumed to be in pixels, however,
        if you pass a value in the range [0, 1] it will be interpreted as a percentage of the image size.
    """
    import ouster.sdk.core.frame_ops as so

    raw_min_v, raw_max_v = indices
    min_v = raw_min_v if raw_min_v is not None else float('-inf')
    max_v = raw_max_v if raw_max_v is not None else float('inf')

    def filter_xyz_iter(frame_set_iter, axis_field, invalid_value, filtered_fields, coord_frame):
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        axis_idx = axis_map[axis_field]
        # construct xyzlut per sensor (lidar and camera share the same XYZLut)
        from ouster.sdk.core import XYZLut, CameraInfo
        dewarp_points = coord_frame == "world"

        def make_lut(info):
            # A camera without pinhole intrinsics / dimensions (e.g. a legacy
            # recording) can't be projected to XYZ; skip XYZ filtering for it.
            if isinstance(info, CameraInfo) and not (
                    info.format.columns_per_frame > 0 and info.format.pixels_per_column > 0
                    and info.model is not None):
                return None
            if coord_frame == "sensor":
                return sensor_frame_xyzlut(info)
            return XYZLut(info)

        xyzluts = [make_lut(s) for s in ctx.frame_set_source.sensor_info]
        for frames in frame_set_iter():
            out = copy.deepcopy(frames)
            for idx, frame in enumerate(out):
                if frame and xyzluts[idx] is not None:
                    # we divide by 1000 since xyzlut values are measured in meters.
                    # filter_xyz applies rolling-shutter dewarp for lidar frames
                    # and a single-pose rigid transform for camera frames when
                    # dewarp_points is set (coord_frame == "world").
                    so.filter_xyz(frame, xyzluts[idx], axis_idx,
                              min_v / 1000, max_v / 1000, invalid_value,
                              filtered_fields=filtered_fields,
                              dewarp_points=dewarp_points)
            yield out

    def filter_field_uv(frame_set_iter, axis_field, invalid_value, filtered_fields):
        for frames in frame_set_iter():
            out = copy.deepcopy(frames)
            for frame in out:
                if frame:
                    so.filter_uv(frame, axis_field, min_v, max_v, invalid_value,
                                 filtered_fields=filtered_fields)
            yield out

    def filter_field_iter(frame_set_iter, axis_field, invalid_value, filtered_fields):
        for frames in frame_set_iter():
            out = copy.deepcopy(frames)
            for frame in out:
                if frame:
                    so.filter_field(frame, axis_field, min_v, max_v, invalid_value,
                                    filtered_fields=filtered_fields)
            yield out

    axis_field = axis_field.strip()
    axis_field_lower = axis_field.lower()
    coord_frame = coord_frame.strip().lower()
    field_list = None if filtered_fields is None else filtered_fields.strip().split(',')

    if axis_field_lower not in {"x", "y", "z"} and coord_frame != "body":
        raise click.BadParameter("is only supported when filtering by X, Y, or Z.",
                                 param_hint="--coord-frame")

    if axis_field_lower in {"x", "y", "z"}:
        ctx.frame_set_iter = partial(filter_xyz_iter, ctx.frame_set_iter, axis_field_lower,
                                invalid_value, field_list, coord_frame)  # type: ignore
    elif axis_field_lower in {"u", "v"}:
        ctx.frame_set_iter = partial(filter_field_uv, ctx.frame_set_iter, axis_field_lower,
                                invalid_value, field_list)  # type: ignore
    else:   # assume it's a field name
        ctx.frame_set_iter = partial(filter_field_iter, ctx.frame_set_iter, axis_field,
                                invalid_value, field_list)  # type: ignore


@click.command
@click.option('-n',
              type=int,
              default=0,
              help="Index of lidar to print metadata of",
              show_default=True)
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def default_source_sensor_info(ctx: SourceCommandContext, click_ctx: click.core.Context, n: int) -> None:
    """
    Display sensor info about the SOURCE.
    """
    source_sensor_info(ctx, click_ctx, n)


def source_sensor_info(ctx: SourceCommandContext, click_ctx: click.core.Context, n: int) -> None:
    src = open_source(ctx.source_uri, **ctx.source_options, collate=False)

    if n >= len(src.sensor_info) or n < 0:
        raise click.ClickException(f"Sensor index {n} out of range. Must be between 0 and {len(src.sensor_info) - 1}")

    print(src.sensor_info[n].to_json_string())


_ALIGN_FRAME_FILL_WINDOW = 10
_ALIGN_MIN_VALID_COLUMN_FRACTION = 0.9


# Minimum valid RANGE points a camera frame needs to participate in alignment.
_ALIGN_MIN_CAMERA_POINTS = 100


def _frame_point_cloud_quality(frame: LidarFrame) -> Tuple[int, float]:
    try:
        ranges = np.asarray(frame.field(ChanField.RANGE))
        valid_cols = np.asarray(frame.status) != 0
        # Measure column fill against the configured azimuth (column) window
        # rather than the full frame width, so sensors running a reduced FOV
        # aren't penalized for columns that lie outside their window.
        expected_cols = frame.sensor_info.format.valid_columns_per_frame()
        valid_col_fraction = np.count_nonzero(valid_cols) / expected_cols
        if (ranges.ndim == 2 and valid_cols.ndim == 1 and
                ranges.shape[1] == valid_cols.size):
            ranges = ranges[:, valid_cols]
        return int(np.count_nonzero(ranges > 0)), valid_col_fraction
    except IndexError:
        return 0, 0.0


def _camera_frame_point_cloud_quality(frame: CameraFrame) -> Tuple[int, float]:
    """Return (valid RANGE point count, valid RANGE fraction) for a camera.

    Cameras have no azimuth columns, so alignment quality is measured by the
    number of pixels carrying a usable (non-zero) range return.
    """
    try:
        rng = np.asarray(frame.field("RANGE"))
    except (IndexError, KeyError):
        return 0, 0.0
    valid = rng != 0
    point_count = int(np.count_nonzero(valid))
    fraction = point_count / rng.size if rng.size else 0.0
    return point_count, fraction


def _frame_has_valid_point_cloud(
        frame: Optional[core.Frame]) -> Tuple[bool, int, float]:
    if frame is None:
        return False, 0, 0.0
    if isinstance(frame, CameraFrame):
        point_count, valid_fraction = _camera_frame_point_cloud_quality(frame)
        return point_count >= _ALIGN_MIN_CAMERA_POINTS, point_count, valid_fraction
    if isinstance(frame, LidarFrame):
        point_count, valid_col_fraction = _frame_point_cloud_quality(frame)
        return (
            valid_col_fraction >= _ALIGN_MIN_VALID_COLUMN_FRACTION,
            point_count,
            valid_col_fraction,
        )
    return False, 0, 0.0


def _frame_extrinsic_info(frame: core.Frame) -> Optional[core.SensorInfo]:
    """Return the SensorInfo carrying a frame's ``sensor_to_body`` extrinsic."""
    return frame.sensor_info


def _plumb_extrinsic_from_saved_z_axis(extrinsic: np.ndarray) -> np.ndarray:
    """Return a rotation-only plumb prior from the saved extrinsic Z axis.

    This is used only by hidden/debug --align-ext-z. It intentionally ignores
    saved yaw and translation so align_clouds can test a Z-axis-only plumb prior.
    Standard align uses the full saved sensor_to_body as its initial prior.
    """
    saved = np.asarray(extrinsic, dtype=np.float64)
    sensor_from_world_z = saved[:3, :3].T @ np.array(
        [0.0, 0.0, 1.0], dtype=np.float64)
    norm = np.linalg.norm(sensor_from_world_z)
    if not np.isfinite(norm) or norm <= 0.0:
        return np.eye(4, dtype=np.float64)
    sensor_from_world_z = sensor_from_world_z / norm
    rotation_matrix = np.asarray(
        get_rot_matrix_to_align_to_gravity(
            float(sensor_from_world_z[0]),
            float(sensor_from_world_z[1]),
            float(sensor_from_world_z[2]),
            False),
        dtype=np.float64)
    extrinsic = np.eye(4, dtype=np.float64)
    extrinsic[:3, :3] = rotation_matrix
    return extrinsic


@click.command(name="align")
@click.option("--align-ext-z",
              is_flag=True,
              default=False,
              hidden=True,
              help=("Debug option: align extrinsics along the saved "
                    "sensor_info.sensor_to_body Z axis only; ignore saved yaw and "
                    "translation. Use this with good existing extrinsics as an "
                    "alternative for data without IMU when testing "
                    "align_clouds."))
@click.pass_context
@source_multicommand(type=SourceCommandType.SINGLE_CONSUMER)
def source_align(ctx: SourceCommandContext,
                 align_ext_z: bool) -> None:
    """Align multi-sensor frames and print aligned extrinsic matrices."""
    frame_set_iter = ctx.frame_set_iter
    frame_set_source = ctx.frame_set_source
    if frame_set_iter is None or frame_set_source is None:
        raise click.ClickException("'align' must be used after a source is initialized.")

    sensor_info = list(frame_set_source.sensor_info)
    num_sensors = len(sensor_info)
    if num_sensors < 2:
        click.secho("Warning: align requires at least two sensors; skipping.",
                    fg='yellow')
        return

    num_frames = 0
    selected = FrameSet([None] * num_sensors)
    selected_from: List[Optional[int]] = [None] * num_sensors
    point_counts: List[int] = [0] * num_sensors
    valid_col_fractions: List[float] = [0.0] * num_sensors

    for frame_set_idx, frames in enumerate(frame_set_iter()):  # type: ignore[operator]
        num_frames += 1
        for sensor_idx, frame in enumerate(frames):
            if sensor_idx >= num_sensors:
                continue
            usable, point_count, valid_col_fraction = (
                _frame_has_valid_point_cloud(frame))
            if not usable:
                continue
            # Keep the most recent usable frame in the fill window for each
            # sensor. This reduces time skew when another sensor appears later
            # in the window.
            selected[sensor_idx] = frame
            selected_from[sensor_idx] = frame_set_idx
            point_counts[sensor_idx] = point_count
            valid_col_fractions[sensor_idx] = valid_col_fraction
        if num_frames >= _ALIGN_FRAME_FILL_WINDOW:
            break
        if all(frame is not None for frame in selected):
            break

    if num_frames == 0:
        click.secho(
            "Warning: align could not read any frame set; skipping.",
            fg='yellow')
    else:
        available: List[Tuple[int, core.Frame]] = [(i, cast(core.Frame, selected[i])) for i in selected.valid_indices()]
        if len(available) < 2:
            click.secho(
                "Warning: align could not find valid frames from at least two "
                f"sensors in the first {num_frames} frame sets; skipping.",
                fg='yellow')
        elif selected[0] is None:
            click.secho(
                "Warning: align requires sensor 0 as the anchor frame; skipping.",
                fg='yellow')
        else:
            for sensor_idx in range(num_sensors):
                source_idx = selected_from[sensor_idx]
                if source_idx is None:
                    click.secho(
                        f"Warning: sensor {sensor_idx} has no valid point "
                        f"cloud in the first {num_frames} frame sets.",
                        fg='yellow')
                elif source_idx != 0:
                    click.echo(
                        f"sensor {sensor_idx}: using frame set {source_idx} "
                        f"({point_counts[sensor_idx]} valid range points, "
                        f"{valid_col_fractions[sensor_idx]:.2f} valid columns)")

            if align_ext_z:
                click.echo(
                    "Using saved extrinsic Z axes for debug alignment "
                    "(saved yaw/translation ignored).")

            alignment_extrinsics = [
                np.asarray(info.sensor_to_body, dtype=np.float64)
                for info in sensor_info
            ]
            if align_ext_z:
                # Hidden debug mode: derive only a plumb rotation from each
                # saved extrinsic Z axis and force translation to zero. The
                # standard path above keeps the full saved extrinsic, including
                # yaw and translation, as the initial alignment prior.
                alignment_extrinsics = [
                    _plumb_extrinsic_from_saved_z_axis(ext)
                    for ext in alignment_extrinsics
                ]
            for sensor_idx, frame in available:
                frame_info = _frame_extrinsic_info(frame)
                if frame_info is not None:
                    frame_info.sensor_to_body = alignment_extrinsics[sensor_idx]
                    # align_clouds projects through the cached xyzlut
                    frame_info.clear_cache()

            click.echo(
                "Aligning sensors to sensor 0 using the first frame set "
                f"with a {_ALIGN_FRAME_FILL_WINDOW}-set fill window")
            try:
                transforms = cast(Iterable[Any], align_clouds(selected))
            except Exception as e:
                click.secho(f"Warning: align failed: {e}; skipping.",
                            fg='yellow')
                transforms = None

            if transforms is not None:
                for sensor_idx, transform in enumerate(transforms):
                    if sensor_idx >= num_sensors:
                        continue
                    selected_frame = selected[sensor_idx]
                    frame_info = (_frame_extrinsic_info(selected_frame)
                                  if selected_frame is not None else None)
                    if frame_info is None:
                        continue
                    transform = np.asarray(transform, dtype=np.float64)
                    click.echo(f"sensor {sensor_idx} aligned extrinsic:")
                    click.echo(np.array2string(
                        transform, precision=6, suppress_small=True))

                    sensor_info[sensor_idx].sensor_to_body = transform
                    frame_info.sensor_to_body = transform
                    # The cached xyzlut bakes in sensor_to_body; drop it so
                    # downstream stages see the aligned extrinsic.
                    sensor_info[sensor_idx].clear_cache()
                    frame_info.clear_cache()

                click.echo("Applied aligned extrinsics to sensor_info.sensor_to_body")


@click.command
@click.option('--method', type=click.Choice(['imu', 'walls', 'all']),
              default='all', show_default=True,
              help="imu: rotate the world so the IMU gravity estimate points "
                   "down. walls: refine the existing orientation from wall "
                   "geometry only, no IMU. all: IMU alignment refined by "
                   "wall geometry, keeping the IMU-only result when the "
                   "scene has too little vertical structure.")
@click.pass_context
@source_multicommand(type=SourceCommandType.SINGLE_CONSUMER)
def source_plumb(ctx: SourceCommandContext, method: str) -> None:
    """Align each sensor's Z axis with gravity.

    Estimates which way is down, from the IMU, from wall geometry, or both
    (see --method), and writes the correction into each sensor's
    sensor_to_body extrinsic. Every sensor is estimated independently from
    its own IMU and its own wall geometry, so a rig whose relative
    extrinsics are not yet established still gets each sensor's Z axis
    levelled; run 'align' afterwards to resolve the remaining relative yaw
    and translation. Nothing is written unless every sensor produces a
    correction. The IMU methods require the FW 3.2+ IMU profile; legacy IMU
    packets are not supported. Both estimates come from the first frame set
    at the current position in the command chain (upstream commands such as
    'slice' move the sampling window). Wall refinement corrects at most
    +/-25 degrees of residual tilt and works best when the sensor is
    stationary at the sampled frame."""

    frame_set_iter = ctx.frame_set_iter
    frame_set_source = ctx.frame_set_source
    if frame_set_iter is None or frame_set_source is None:
        raise click.ClickException(
            "'plumb' must be used after a source is initialized.")
    use_imu = method in ('imu', 'all')
    use_walls = method in ('walls', 'all')

    # Plumb/level each sensor independently. IMU leveling works for both lidar
    # and camera frames (a camera carrying Ouster-format IMU fields, e.g. a ZED
    # saved alongside a lidar); wall refinement is lidar-only (cameras have no
    # RANGE/normals), so a camera is levelled from its IMU alone. infos is the
    # full, frame-order-aligned sensor list so camera streams are included.
    infos = frame_set_source.sensor_info
    num_sensors = len(infos)
    base_exts = [np.asarray(info.sensor_to_body, dtype=np.float64)
                 for info in infos]

    # The IMU deskew (used to sharpen wall normals) is lidar-only in effect, but
    # is built from the full, frame-order-aligned sensor list so its per-sensor
    # state stays index-aligned with the sampled FrameSet regardless of
    # lidar/camera ordering; it skips any camera frames internally.
    lidar_infos = [info for info in infos
                   if isinstance(info, core.LidarInfo)]

    # Sample the first frame set of the chained stream, so upstream commands
    # such as 'slice' apply naturally. The IMU average and the wall
    # extraction both come from this frame set, matching the prototype
    # script.
    stream = iter(frame_set_iter())  # type: ignore[operator]
    sampled = next(stream, None)
    selected: List[Optional[core.Frame]] = (
        list(sampled) if sampled is not None else [None] * num_sensors)

    # Deskew the sampled frames with the same IMU deskew method 'slam' uses
    # (skipped for --method walls, which is strictly IMU-free, and when there
    # are no lidar sensors). The deskewer writes per-column poses into the
    # frames in place, so snapshot them and restore below to hand the frame set
    # downstream pristine.
    pose_backups: List[Optional[np.ndarray]] = []
    if use_imu and sampled is not None and len(lidar_infos) > 0:
        pose_backups = [
            np.asarray(frame.body_to_world).copy() if frame is not None
            else None
            for frame in selected]
        # IMU plumb only: the wall stage below runs on its own.
        deskewer = mapping.DeskewMethodFactory.create('imu_deskew', infos, 'imu')
        deskewer.update(sampled)

    # Estimate per sensor: each sensor is levelled from its own IMU and its
    # own wall geometry, so a rig whose relative extrinsics are not yet
    # established still gets every sensor's Z axis levelled. Nothing is
    # written until every sensor has a correction, so a partial failure
    # leaves the extrinsics untouched rather than half-levelled.
    corrections: List[Optional[np.ndarray]] = [None] * num_sensors
    indent = "  " if num_sensors > 1 else ""

    for sensor_index, frame in enumerate(selected):
        sn = infos[sensor_index].sn
        if num_sensors > 1:
            click.echo(f"Sensor {sn}:")
        if frame is None:
            click.secho(
                f"{indent}ERROR: sensor {sn} has no frame in the sampled "
                "frame set.", fg='red')
            continue
        # Both stages on this sensor alone: its own IMU, its own wall geometry.
        # The stages log what they found; only the outcome is reported here.
        correction = plumb(FrameSet([frame]), use_imu, use_walls)
        if correction is None:
            click.secho(
                f"{indent}ERROR: could not plumb sensor {sn}; see the log "
                "for the reason.", fg='red')
            continue
        corrections[sensor_index] = np.asarray(correction, dtype=np.float64)

    if any(correction is None for correction in corrections):
        click.secho(
            "ERROR: plumb could not level every sensor; sensor extrinsics "
            "left unchanged.", fg='red')
        # Finalize the peeked stream before aborting: many CLI invocations
        # share one process under pytest, and an abandoned generator would
        # print its progress-bar cleanup into a later command's stdout at
        # GC time.
        stream.close()
        exit(1)

    for sensor_index, base_ext in enumerate(base_exts):
        correction = corrections[sensor_index]
        assert correction is not None
        label = (f"Plumb correction for sensor {infos[sensor_index].sn}:"
                 if num_sensors > 1 else "Plumb correction:")
        click.echo(label)
        click.echo(np.array2string(correction, precision=6,
                                   suppress_small=True))
        infos[sensor_index].sensor_to_body = correction @ base_ext
        # The cached xyzlut bakes in sensor_to_body; drop it so downstream
        # stages see the corrected extrinsic.
        infos[sensor_index].clear_cache()

    # The correction is written into calibration, not into the frames, so say
    # so plainly; the failure path above has the matching "left unchanged".
    if num_sensors > 1:
        click.echo(
            f"Updated the sensor_to_body extrinsic of {num_sensors} sensors. "
            "Each sensor was levelled independently, so the relative yaw and "
            "translation between them is unchanged; run 'align' if that is "
            "not already established.")
    else:
        click.echo("Updated the sensor_to_body extrinsic.")

    if sampled is not None:
        if pose_backups:
            # Restore the sampled frames' poses before re-yielding them (the
            # backups exist only when the IMU stage deskewed them in place).
            for frame, backup in zip(selected, pose_backups):
                if frame is not None and backup is not None:
                    np.asarray(frame.body_to_world)[:] = backup

        # Hand back an unconsumed stream: the first pass replays the sampled
        # frame set followed by the rest, later passes (e.g. viz loop mode)
        # restart fresh.
        first_pass = {'pending': True}

        def plumb_iter():
            if first_pass['pending']:
                first_pass['pending'] = False
                yield sampled
                yield from stream
            else:
                yield from frame_set_iter()

        ctx.frame_set_iter = plumb_iter  # type: ignore[assignment]


@click.command
@click.argument('output_osf_file', required=True, type=str)
@click.option('--viz', is_flag=True,
              help="Launch interactive pose optimization visualization instead of batch solving.")
@click.option('--no-initial-align', is_flag=True, default=False,
              help="Disable the initial alignment with absolute constraints in the visualizer.")
@click.option('--config', 'constraints_json_file', required=False, type=str, default=None,
              help="Constraints JSON configuration file to load.")
@click.option('--auto-gps', 'auto_constraints', is_flag=True, default=False,
              help="Automatically generate and add GPS absolute pose constraints "
                   "(in addition to any constraints loaded from '--config').")
@click.option('--auto-loop', is_flag=True, default=False,
              help="Automatically generate and add loop-closure pose constraints.")
@click.option('--gps-constraints-every-m', type=click.FloatRange(min=0, min_open=True),
              default=100.0, show_default=True,
              help="Distance in meters between GPS absolute pose constraints to add when "
                   "'--auto-gps' is set. Distance is computed from lidar frame poses.")
@click.option('--gps-constraints-weights', type=str, default="0.01,0.01,0.001", show_default=True,
              help="Translation weights WX,WY,WZ for auto-generated GPS absolute pose constraints. "
                   "The default value constrains GPS-derived position X,Y and softly constrains Z with the "
                   "lidar frame pose Z (from SLAM) when available.")
@click.option('--loop-min-distance-m', type=click.FloatRange(min=1, min_open=False),
              default=50.0, show_default=True,
              help="Min traveled distance (meters) required between successive "
                   "loop-closure additions.")
@click.option('--loop-cell-size-m', type=click.FloatRange(min=1, min_open=False),
              default=None, show_default="auto",
              help="Spatial hash grid cell size in meters for loop detection. "
                   "If not provided, auto calculate it.")
@click.option('--icp-threshold',
              type=click.FloatRange(min=0.0, max=1.0),
              default=None, show_default="0.6 (batch), 0.0 (viz)",
              help="Min auto-ICP confidence score in [0,1] required to keep "
                   "auto-loop pairs. If omitted: 0.6 in batch mode, 0.0 in viz mode.")
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED)
def source_pose_optimize(ctx, output_osf_file: str, viz: bool, no_initial_align: bool,
                         constraints_json_file: Optional[str], auto_constraints: bool,
                         auto_loop: bool, gps_constraints_every_m: float,
                         gps_constraints_weights: Optional[str],
                         loop_min_distance_m: float,
                         loop_cell_size_m: Optional[float],
                         icp_threshold: Optional[float]) -> None:
    """
    Optimizes the SLAM trajectory by refining the poses based on the provided constraints.

    \b
    Examples:
      ouster-cli source <OSF_FILENAME> pose_optimize --viz --config <CONSTRAINT_JSON_FILE> <OUTPUT_OSF_FILENAME>
      ouster-cli source <OSF_FILENAME> pose_optimize --viz --auto-gps
          <OUTPUT_OSF_FILENAME>
      ouster-cli source <OSF_FILENAME> pose_optimize --viz --auto-gps --config
          <CONSTRAINT_JSON_FILE> <OUTPUT_OSF_FILENAME>
      ouster-cli source <OSF_FILENAME> pose_optimize --auto-gps --gps-constraints-weights
          0.02,0.02,0.001 --gps-constraints-every-m 150 --viz <OUTPUT_OSF_FILENAME>
    """

    # Get source from context (passed as SOURCE argument to the CLI)
    source_name = ctx.source_uri
    if not source_name:
        print("Error: SOURCE must be specified as a command-line argument.")
        return

    if constraints_json_file is not None:
        po = mapping.PoseOptimizer(source_name, constraints_json_file)
    else:
        po = mapping.PoseOptimizer(source_name, 1.0)

    if auto_constraints:
        try:
            click.echo("Generating GPS constraints ...")
            weights = parse_gps_constraints_weights(gps_constraints_weights)
            gps_constraints_count = po.add_absolute_gps_constraints(
                gps_constraints_every_m, weights)
            click.echo(f"Added {gps_constraints_count} GPS absolute pose constraints.")
            if not viz:
                po.initialize_trajectory_alignment()
        except Exception as error:
            click.secho(f"ERROR: Failed to add GPS constraints: {error}", fg="red")
            return

    # If both GPS and loop auto-constraints are requested, first solve with GPS
    # constraints so loop candidates are found on a better trajectory.
    run_two_stage_auto = auto_constraints and auto_loop and (not viz)
    if run_two_stage_auto:
        try:
            click.echo("Solving trajectory with GPS constraints before loop detection ...")
            po.solve()
        except Exception as error:
            click.secho(f"ERROR: Failed to solve after GPS constraints: {error}", fg="red")
            return

    loop_cell_size_m_value = loop_cell_size_m
    if loop_cell_size_m_value is None and (auto_loop or viz):
        from .source_mapping import auto_loop_cell_size_m

        loop_cell_size_m_value = auto_loop_cell_size_m(po)
        if auto_loop:
            click.echo(f"Auto loop cell size: {loop_cell_size_m_value:.2f} m")

    defer_auto_loop_in_viz = viz and auto_constraints and auto_loop
    if icp_threshold is None:
        icp_threshold_value = 0.0 if viz else 0.6
    else:
        icp_threshold_value = float(icp_threshold)
    if auto_loop and not defer_auto_loop_in_viz:
        try:
            click.echo("Generating loop closure constraints ...")
            assert loop_cell_size_m_value is not None
            loop_constraints_count = po.add_relative_loop_constraints(
                loop_min_distance_m, loop_cell_size_m_value,
                icp_threshold_value)
            click.echo(f"Added {loop_constraints_count} loop closure constraints.")
        except Exception as error:
            click.secho(f"ERROR: Failed to add loop constraints: {error}", fg="red")
            return
    elif defer_auto_loop_in_viz:
        click.echo(
            "Deferring auto-loop generation in viz mode; press 'l' in viz to "
            "generate loop closures from current trajectory."
        )

    if viz:
        viewer = PoseOptimizerViz(
            po,
            output_osf_file,
            input_source_uri=source_name,
            align_with_absolute_constraints=not no_initial_align,
            loop_cell_size_m=loop_cell_size_m_value,
            loop_min_distance_m=loop_min_distance_m,
            loop_icp_threshold=icp_threshold_value,
        )
        viewer.run()
        return

    if run_two_stage_auto:
        try:
            click.echo("Solving trajectory with loop constraints ...")
            po.solve()
        except Exception as error:
            click.secho(f"ERROR: Failed to solve after loop constraints: {error}", fg="red")
            return
    else:
        po.solve()
    po.save(output_osf_file)
    print("\nPose Optimization Completed Successfully")


@click.command()
@click.option("-v", "--verbose", is_flag=True, help="Print out additional stats info.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_stats(ctx: SourceCommandContext, verbose: bool) -> None:
    """Calculate and output various statistics about the frames at this point in the pipeline."""
    assert ctx.frame_set_source is not None
    frames = ctx.frame_set_iter
    count = 0
    incomplete_count = 0
    start = None
    end = None
    start_sensor = None
    end_sensor = None
    incomplete_frames = []
    incomplete_count = 0
    dimensions = {}
    n_sensors = len(ctx.frame_set_source.sensor_info)
    totals = {}
    totals["objects in frame sets"] = 0
    totals["objects in frames"] = 0
    total_packets = 0
    objects_in_frames = 0
    objects_in_frame_sets = 0
    for m in ctx.frame_set_source.sensor_info:
        if isinstance(m, core.LidarInfo):
            if m.format.zone_monitoring_enabled:
                totals["zm packets"] = 0
            if m.format.udp_profile_lidar != core.UDPProfileLidar.OFF:
                totals["lidar packets"] = 0
                totals["lidar columns"] = 0
            if m.format.udp_profile_imu != core.UDPProfileIMU.LEGACY:
                totals["imu packets"] = 0

    def stats_iter():
        nonlocal count, start, end, incomplete_count
        nonlocal objects_in_frames, objects_in_frame_sets
        nonlocal start_sensor, end_sensor, total_packets
        ns_to_sec = 1.0 / 1000000000.0
        for index, l in enumerate(frames()):
            if isinstance(l, FrameSet):
                objects_in_frame_sets += sum([len(v) for v in l.objects.values()])
            for i, frame in enumerate(l):
                if frame is None:
                    continue
                try:
                    time = frame.get_min_valid_receive_timestamp()
                    if start is None or time < start:
                        start = time
                    time = frame.get_max_valid_receive_timestamp()
                    if end is None or time > end:
                        end = time
                except RuntimeError:
                    pass
                try:
                    sensor_time = frame.timestamp[frame.get_first_valid_column()]
                    if sensor_time != 0.0:
                        if start_sensor is None or sensor_time < start_sensor:
                            start_sensor = sensor_time
                    sensor_time = frame.timestamp[frame.get_last_valid_column()]
                    if sensor_time != 0.0:
                        if end_sensor is None or sensor_time > end_sensor:
                            end_sensor = sensor_time
                except RuntimeError:
                    pass
                dimensions[(frame.w, frame.h)] = True

                complete = True
                missing_elements = []
                received_packets = np.count_nonzero(frame.receive_timestamp)
                total_packets += received_packets
                if isinstance(frame, LidarFrame) and not frame.complete() \
                    and frame.sensor_info.format.udp_profile_lidar != core.UDPProfileLidar.OFF:
                    expected_columns = frame.sensor_info.format.valid_columns_per_frame()
                    expected_packets = frame.sensor_info.format.lidar_packets_per_frame()
                    received_columns = np.count_nonzero(frame.status & 1)
                    received_packets = np.count_nonzero(frame.receive_timestamp)
                    m_columns = expected_columns - received_columns
                    m_packets = expected_packets - received_packets
                    totals["lidar packets"] += m_packets
                    totals["lidar columns"] += m_columns
                    complete = False
                    if verbose:
                        missing_elements.append(f", missing {m_packets} lidar packets")
                        missing_elements.append(f", missing {m_columns} lidar columns")

                # check imu packets
                if frame.has_field("IMU_PACKET_TIMESTAMP"):
                    data = frame.field("IMU_PACKET_TIMESTAMP")
                    cnt = np.count_nonzero(data)
                    m_packets = len(data) - cnt
                    total_packets += cnt
                    if m_packets > 0:
                        complete = False
                        totals["imu packets"] += m_packets
                        missing_elements.append(f", missing {m_packets} imu packets")

                # check zone packets
                if frame.has_field("ZONE_PACKET_TIMESTAMP"):
                    data = frame.field("ZONE_PACKET_TIMESTAMP")
                    cnt = np.count_nonzero(data)
                    m_packets = len(data) - cnt
                    total_packets += cnt
                    if m_packets > 0:
                        complete = False
                        totals["zm packets"] += m_packets
                        missing_elements.append(f", missing {m_packets} zone packets")

                objects_in_frames += sum([len(v) for v in frame.objects.values()])

                if not complete:
                    incomplete_count = incomplete_count + 1

                if verbose and missing_elements:
                    string = f"    #{index} sensor {i} at {time * ns_to_sec}"
                    for element in missing_elements:
                        string = string + element
                    incomplete_frames.append(string)
                count = count + 1
            yield l
    ctx.frame_set_iter = stats_iter  # type: ignore

    def exit_handler():
        ns_to_sec = 1.0 / 1000000000.0
        print("Frame Statistics:")
        print(f"  Count: {count}")
        print(f"  Packet Count: {total_packets}")
        print(f"  Sensors: {n_sensors}")
        dstring = ""
        for k in dimensions:
            dstring = dstring + f" {k[0]}x{k[1]}"
        print(f"  Sizes:{dstring}")
        if start is None:
            print("  First Receive Time: No Valid Timestamps")
            print("  Last Receive Time: No Valid Timestamps")
            print("  Duration: Unknown")
        else:
            print(f"  First Receive Time: {start * ns_to_sec} ({_nanos_to_string(start)})")
            print(f"  Last Receive Time: {end * ns_to_sec} ({_nanos_to_string(end)})")
            print(f"  Duration: {(end - start) * ns_to_sec} seconds")
        if start_sensor is None:
            print("  First Sensor Time: No Valid Timestamps")
            print("  Last Sensor Time: No Valid Timestamps")
        else:
            print(f"  First Sensor Time: {start_sensor * ns_to_sec} ({_nanos_to_string(start_sensor)})")
            print(f"  Last Sensor Time: {end_sensor * ns_to_sec} ({_nanos_to_string(end_sensor)})")
        print(f"  Objects in Frame Sets: {objects_in_frame_sets}")
        print(f"  Objects in Frames: {objects_in_frames}")
        line = f"  Incomplete Frames: {incomplete_count}"
        for key in totals:
            line = line + f", {totals[key]} missing {key}"
        print(line)
        if verbose:
            for i in incomplete_frames:
                print(i)
        elif incomplete_count > 0:
            print("Notice: Rerun using 'stats -v' for more details about incomplete frames.")

    atexit.register(exit_handler)


def reject_camera_streams(frame_set_source: FrameSetSource, command: str) -> None:
    """Raise if the source contains camera streams, which ``command`` can't handle."""
    for i, m in enumerate(frame_set_source.sensor_info):
        if isinstance(m, core.CameraInfo):
            raise click.exceptions.UsageError(
                f"{command} is not compatible with sources containing camera streams,"
                f" but source[{i}] is a camera.")


@click.command
@click.argument('beams', required=True,
              type=click.Choice(['1', '2', '4', '8', '16', '32', '64', '128']))
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_reduce(ctx: SourceCommandContext, beams: str, **kwargs) -> None:
    """
    Reduce the number of beams for each source to the specified beam count
    """
    assert ctx.frame_set_source is not None
    reject_camera_streams(ctx.frame_set_source, "reduce")
    # validate input (beam reduction is lidar-only)
    for i, m in enumerate(ctx.frame_set_source.sensor_info):
        if not isinstance(m, core.LidarInfo):
            continue
        if int(beams) > m.format.columns_per_frame:
            raise click.exceptions.UsageError(
                f"selected beams count can't be larger than input, source[{i}] has"
                f" a beam count of {m.format.pixels_per_column}, but {beams} selected")


def parse_beam_indices(indices: str) -> List[int]:
    try:
        parsed_indices = [int(i.strip()) for i in indices.split(',')]
    except ValueError:
        raise click.exceptions.BadParameter(
            "beam indices must be a comma separated list of integers")

    if not parsed_indices:
        raise click.exceptions.BadParameter("beam indices can't be empty")
    if len(parsed_indices) != len(set(parsed_indices)):
        raise click.exceptions.BadParameter("beam indices can't contain duplicates")
    return parsed_indices


@click.command
@click.argument('indices', required=True, type=str)
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_select(ctx: SourceCommandContext, indices: str, **kwargs) -> None:
    """
    Select specific beams by their comma separated indices.
    """
    assert ctx.frame_set_source is not None
    reject_camera_streams(ctx.frame_set_source, "select")
    parse_beam_indices(indices)


def _solid_color(rgb: Tuple[float, float, float], n: int) -> np.ndarray:
    return np.full((n, 3), rgb, dtype=np.float32)


_SOLID_COLORS: Tuple[Tuple[str, Tuple[float, float, float]], ...] = (
    ("red", (1.0, 0.0, 0.0)),
    ("green", (0.0, 1.0, 0.0)),
    ("blue", (0.0, 0.0, 1.0)),
    ("yellow", (1.0, 1.0, 0.0)),
    ("cyan", (0.0, 1.0, 1.0)),
    ("magenta", (1.0, 0.0, 1.0)),
)


@click.command
@click.option("-c", "--center/--no-center", default=True,
              help="Center the view on the median of first pointcloud. Useful for clouds"
                   " that may be offset or in coordinate frames such as ECEF.",
              show_default=True)
@click.option("-b", "--background", default="gray",
              help="Background color. Accepts hex codes or simple color names."
                   " (e.g. 0x777777, #888 or red)",
              show_default=True, callback=parse_color)
@click.option("--ae", is_flag=True, help="Apply basic auto exposure to color channels.")
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED)
def model_viz(ctx: SourceCommandContext, center: bool,
              background: Tuple[float, float, float, float],
              ae: bool, **kwargs) -> None:
    """
    View a pointcloud or stl file. Multiple PLY, PCD, and/or LAS/LAZ files can be
    combined into a single view by providing them as a comma separated SOURCE list
    (or via '--glob'), each loaded as its own point cloud and, when more than one is loaded,
    defaulted to a distinct solid color for easy visual separation, up to a maximum of six
    pointclouds shown side-by-side. Press number keys 1-6 to toggle the visibility of each
    loaded pointcloud, and CTRL+number (add SHIFT to go backwards) to cycle its coloring mode.
    Press 'm' (add SHIFT to go backwards) to cycle every loaded pointcloud through the same
    color channel together (a cloud that lacks that particular channel is drawn solid black).
    Press 'p' to change point size. Press '9' to toggle the origin axes.
    """
    from ouster.sdk import viz
    from ouster.cli.plugins.read_pointcloud_color import read_pointcloud, LoadedCloud

    from ouster.sdk.core import Stl

    pviz = viz.PointViz("Model Viewer")
    viz.add_default_controls(pviz)
    pviz.notifications_enabled = True

    source_list = ctx.source_uris

    center_pt = np.array((0.0, 0.0, 0.0))

    pviz.set_background_color(background)

    # map origin axis, mirroring LidarFrameViz's map-origin indicator in viz/core.py;
    # toggled with key '9', mirroring LidarFrameViz.toggle_axis_markers
    map_origin_axis = viz.util.AxisWithLabel(pviz, label="M", thickness=5, label_scale=0.4)

    def shared_key_handlers(ctx, key, mod):
        if chr(key) == '9' and mod == 0:
            map_origin_axis.toggle()
            pviz.set_notification(
                f"Origin axes: {'shown' if map_origin_axis.enabled else 'hidden'}")
            pviz.update()
            return False
        if chr(key) == "Z" and mod == 1:
            file_name = pviz.save_screenshot("")
            if file_name:
                pviz.set_notification(f"Saved screenshot to: {file_name}")
            pviz.update()
            return False
        return True

    pviz.push_key_handler(shared_key_handlers)

    is_mesh = bool(source_list) and all(p.endswith(".stl") for p in source_list)
    is_cloud = bool(source_list) and all(p.endswith((".ply", ".pcd", ".las", ".laz")) for p in source_list)

    help_text = ">---------------- Key Bindings --------------<\n"
    help_text += "9: Toggle origin axes\n"
    help_text += "SHIFT+Z: Save screenshot"
    if is_cloud:
        help_text += "\nSHIFT+P/P: Decrease/increase point size.\n"
        help_text += "1-6: Toggle visibility of the Nth loaded pointcloud.\n"
        help_text += "CTRL+1-6: Cycle color mode of the Nth loaded pointcloud.\n"
        help_text += "CTRL+SHIFT+1-6: Cycle color mode of the Nth loaded pointcloud backwards.\n"
        help_text += "SHIFT+M/M: Cycle every loaded pointcloud through the same color "
        help_text += "channel (black where a cloud lacks it).\n"
        help_text += "CTRL+E: Enable/disable eye-dome lighting."

    help_shown = False
    help_label = viz.Label(help_text, 0.0, 1.0, align_right=False, align_top=False)
    help_label.set_scale(1.5)

    def toggle_help():
        nonlocal help_shown
        help_shown = not help_shown
        if help_shown:
            print(help_text)
            pviz.add(help_label)
        else:
            pviz.remove(help_label)
        pviz.update()

    if is_mesh:
        for i, path in enumerate(source_list):
            mesh = Stl(path).to_mesh()
            if i == 0 and len(mesh.triangles) > 0:
                center_pt = mesh.triangles[0].coords[0]
            m = viz.Mesh.from_simple_mesh(mesh)
            pviz.add(m)

        def key_handler(ctx, key, mod):
            # filter out invalid keys (e.g. function keys) that don't correspond to a valid Unicode codepoint
            if key < 0 or key > 0x10FFFF:
                return False
            ch = chr(key)
            handled = False
            if ch == "/" and mod == 1:
                toggle_help()
                handled = True
            return not handled

        pviz.push_key_handler(key_handler)

    elif is_cloud:
        MAX_SIDE_BY_SIDE_CLOUDS = 6
        if len(source_list) > MAX_SIDE_BY_SIDE_CLOUDS:
            click.secho(
                f"Warning: only the first {MAX_SIDE_BY_SIDE_CLOUDS} pointclouds can be viewed "
                f"side-by-side; the remaining {len(source_list) - MAX_SIDE_BY_SIDE_CLOUDS} "
                f"will not be loaded.",
                fg="yellow")
            source_list = source_list[:MAX_SIDE_BY_SIDE_CLOUDS]

        # shared offset (the median of the first pointcloud) so that all clouds stay
        # aligned relative to one another after centering
        origin_offset: Optional[np.ndarray] = None

        clouds: List[Dict[str, Any]] = []
        for i, path in enumerate(source_list):
            data, _ = read_pointcloud(path)
            loaded = LoadedCloud(data)
            pts = loaded.get_xyz()

            # offset the points rather than the camera in case they are very far from origin
            # this greatly improves visualization precision (since it uses floats)
            if center and len(pts) > 0:
                if origin_offset is None:
                    origin_offset = np.nanmedian(pts, axis=0)  # using the component median as the center
                    print(f"Centering on {origin_offset}")
                    new_center = np.eye(4)
                    new_center[:3, 3] = -origin_offset
                    map_origin_axis.pose = new_center
                pts[:] -= origin_offset

            # convert to float after centering to save memory
            pts = np.ascontiguousarray(pts, np.float32)

            cld = viz.Cloud(pts.shape[0])
            cld.set_xyz(pts)

            # build list of channels we can color by
            channels = []
            rgb = loaded.get_rgb(ae)
            if rgb is not None:
                channels.append(("rgb", rgb))
            normals = loaded.get_normal_colors()
            if normals is not None:
                channels.append(("normals", normals))
            for name, fun in loaded.get_all_scalar_colors():
                channels.append((name, fun))
            channels.append(("xyz", loaded.get_xyz_colors(pts)))
            n_pts = pts.shape[0]
            # Only add the solid colors if there are multiple pointclouds loaded, and
            # default to a distinct one per cloud (cycling through the palette) so that
            # multiple loaded pointclouds are easy to visually tell apart from the start.
            default_channel = 0
            if len(source_list) > 1:
                default_channel = len(channels) + (i % len(_SOLID_COLORS))
                for name, solid_rgb in _SOLID_COLORS:
                    channels.append((name, partial(_solid_color, solid_rgb, n_pts)))

            cld.set_key(channels[default_channel][1]())

            # map channel name -> color function so the master ('m') cycle can
            # look a cloud's color up by name (first occurrence wins on collision)
            channel_by_name: Dict[str, Any] = {}
            for cname, cfun in channels:
                channel_by_name.setdefault(cname, cfun)

            pviz.add(cld)
            clouds.append({
                "name": os.path.basename(path),
                "cloud": cld,
                "channels": channels,
                "channel_by_name": channel_by_name,
                # solid black, used when this cloud lacks the master channel
                "black": partial(_solid_color, (0.0, 0.0, 0.0), n_pts),
                "curr_channel": default_channel,
                "visible": True,
            })

        # superset of color channel names across every loaded pointcloud, in
        # first-seen order; the master ('m') cycle steps through this list
        all_channel_names: List[str] = []
        seen_channel_names = set()
        for info in clouds:
            for cname, _ in info["channels"]:
                if cname not in seen_channel_names:
                    seen_channel_names.add(cname)
                    all_channel_names.append(cname)

        if len(clouds) == 1:
            pviz.set_title(f"Model Viewer: {source_list[0]}")
        else:
            pviz.set_title(f"Model Viewer: {len(clouds)} pointclouds")

        point_size = 3
        max_point_size = 10
        eye_dome = False
        # index into all_channel_names for the master ('m') cycle; -1 so the
        # first forward press lands on the first channel
        master_channel_idx = -1
        for info in clouds:
            info["cloud"].set_point_size(point_size)

        def key_handler(ctx, key, mod):
            nonlocal point_size, eye_dome, master_channel_idx
            # filter out invalid keys (e.g. function keys) that don't correspond to a valid Unicode codepoint
            if key < 0 or key > 0x10FFFF:
                return False
            ch = chr(key)
            handled = False
            if ch == 'P':
                if mod == 1:
                    point_size = max(1, point_size - 1)
                else:
                    point_size = min(max_point_size, point_size + 1)
                for info in clouds:
                    info["cloud"].set_point_size(point_size)
                pviz.set_notification(f"Point Size: {point_size}")
                pviz.update()
                handled = True
            elif ch in "123456" and mod in (0, 2, 3):
                idx = int(ch) - 1
                if idx < len(clouds):
                    info = clouds[idx]
                    if mod in (2, 3):
                        direction = -1 if mod == 3 else 1
                        info["curr_channel"] = (info["curr_channel"] + direction) % len(info["channels"])
                        channel = info["channels"][info["curr_channel"]]
                        info["cloud"].set_key(channel[1]())
                        pviz.set_notification(f"{info['name']}: {channel[0]}")
                    else:
                        info["visible"] = not info["visible"]
                        if info["visible"]:
                            pviz.add(info["cloud"])
                        else:
                            pviz.remove(info["cloud"])
                        pviz.set_notification(
                            f"{info['name']}: {'shown' if info['visible'] else 'hidden'}")
                    pviz.update()
                    handled = True
            elif ch == 'M' and all_channel_names:
                # cycle every cloud onto the *same* named color channel; clouds
                # that don't have that channel fall back to solid black
                direction = -1 if mod == 1 else 1
                master_channel_idx = (master_channel_idx + direction) % len(all_channel_names)
                name = all_channel_names[master_channel_idx]
                for info in clouds:
                    fun = info["channel_by_name"].get(name)
                    if fun is not None:
                        info["cloud"].set_key(fun())
                        # keep per-cloud state in sync for CTRL+number cycling
                        for ci, (cname, _) in enumerate(info["channels"]):
                            if cname == name:
                                info["curr_channel"] = ci
                                break
                    else:
                        info["cloud"].set_key(info["black"]())
                pviz.set_notification(f"All pointclouds: {name}")
                pviz.update()
                handled = True
            elif ch == "E" and mod == 2:
                eye_dome = not eye_dome
                pviz.enable_eye_dome_lighting(eye_dome)
                if eye_dome:
                    pviz.set_notification("Enabled eye-dome lighting")
                else:
                    pviz.set_notification("Disabled eye-dome lighting")
                handled = True
            elif ch == "/" and mod == 1:
                toggle_help()
                handled = True
            # returning False stops the key event from also reaching the default
            # camera controls (e.g. plain '0' normally toggles orthographic mode)
            return not handled

        pviz.push_key_handler(key_handler)
    else:
        click.secho("Unsupported file type. Only .stl, .ply, .pcd, .las, and .laz are supported. "
                    "When providing multiple files they must all be the same kind "
                    "(all meshes or all pointclouds).", fg="red")
        exit(1)

    if center:
        camera_pose = np.eye(4)
        camera_pose[:3, 3] = center_pt
        pviz.camera.set_target(np.linalg.inv(camera_pose))

    pviz.update()
    pviz.run()


@click.command
@click.pass_context
@click.argument("filename", required=True)
@click.option('-d', '--downsample', default=None, type=float,
              help="Downsample voxel size in meters.")
@click.option('--strategy', default='RANDOM',
              type=click.Choice(['FIRST_N_POINT', 'LAST_N_POINT', 'AVERAGE_POINT', 'RANDOM', 'SMALLEST',
                                'LARGEST']),
              help="Downsample strategy.")
@click.option('--attribute-index', default=0, type=click.IntRange(0, 2), show_default=True,
              help=("Which color channel to rank by: 0=red, 1=green, 2=blue. Color is the only"
                    " per-point attribute model_save has to rank with (it reads xyz only). Only"
                    " used, and required to be meaningful, when --strategy is SMALLEST or LARGEST."))
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing file with the same name.")
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED)
def model_save(ctx: SourceCommandContext, filename: str, downsample: Optional[float],
               strategy: Optional[str], attribute_index: int, overwrite: bool, **kwargs) -> None:
    """
    Resave a pointcloud file with optional downsampling.
    """
    import numpy as np
    from ouster.sdk.core import voxel_downsample, VoxelDownsampleStrategy
    from ouster.cli.plugins.read_pointcloud_color import read_pointcloud_color
    from ouster.cli.plugins.map_export import save_pointcloud

    if not overwrite and os.path.isfile(filename):
        click.echo(f"Error: File '{filename}' already exists. Add --overwrite "
                   "flag to overwrite and continue anyways.")
        exit(2)
    xyz, rgb = read_pointcloud_color(ctx.source_uri)
    pts = xyz  # no support for other fields yet

    strategies = {
        'FIRST_N_POINT': VoxelDownsampleStrategy.FIRST_N_POINT,
        'LAST_N_POINT': VoxelDownsampleStrategy.LAST_N_POINT,
        'AVERAGE_POINT': VoxelDownsampleStrategy.AVERAGE_POINT,
        'RANDOM': VoxelDownsampleStrategy.RANDOM,
        'SMALLEST': VoxelDownsampleStrategy.SMALLEST,
        'LARGEST': VoxelDownsampleStrategy.LARGEST,
    }
    selected_strategy = strategies[strategy or 'RANDOM']

    # todo save/downsample other fields when read_pointcloud supports them. Color
    # (always populated by read_pointcloud_color, real or derived) is the only
    # per-point data available for SMALLEST/LARGEST to rank by until then; only
    # build and carry it through for those two, so the other strategies keep
    # using the cheaper positions-only path.
    if downsample is not None:
        if selected_strategy in (VoxelDownsampleStrategy.SMALLEST, VoxelDownsampleStrategy.LARGEST):
            attributes = [np.ascontiguousarray(rgb[:, i]) for i in range(3)]
            pts, _ = voxel_downsample(pts, downsample, strategy=selected_strategy,
                                      attributes=attributes, attribute_index=attribute_index)
        else:
            pts, _ = voxel_downsample(pts, downsample, strategy=selected_strategy)

    # TODO[UN]: Add support for saving other fields when they exist
    save_pointcloud(filename, pts)


@click.command
@click.argument("image_path", required=True,
              type=click.Path(exists=True, dir_okay=False))
@click.option('--fields', default=None, type=str,
              help="A comma separated list of field names to narrow down which fields"
                   " the mask will be applied to.")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR)
def source_mask(ctx: SourceCommandContext, image_path: str, fields: str, **kwargs) -> None:
    """
    Applies a 2D mask, read from a PNG image, to all streamed lidar frames
    """
    import ouster.sdk.core.frame_ops as so
    from ouster.sdk.core.data import destagger
    from ouster.sdk.io import read_png

    assert ctx.frame_set_source is not None

    try:
        image = read_png(image_path)
    except RuntimeError as e:
        raise click.ClickException(f"Failed to read mask image, which must be a PNG: {e}")

    if image.ndim == 3:
        click.secho(f"image [{image_path}] is not an 8-bit grayscale,"
                    " performing conversion", fg="yellow")
        if image.shape[2] >= 3:
            # ITU-R 601-2 luma weights in 16 bit fixed point; alpha is ignored
            rgb = image[..., :3].astype(np.uint32)
            image = ((rgb[..., 0] * 19595 + rgb[..., 1] * 38470 + rgb[..., 2] * 7471
                      + 0x8000) >> 16).astype(np.uint8)
        else:
            image = image[..., 0]

    # validate input (masking/destaggering is lidar-only). Build one mask per
    # sensor slot, index-aligned with the FrameSet frame slots; non-lidar slots
    # get None and are skipped so a camera in the frame set does not shift the
    # mask indices.
    masks: List[Optional[np.ndarray]] = []
    for i, m in enumerate(ctx.frame_set_source.sensor_info):
        if not isinstance(m, core.LidarInfo):
            masks.append(None)
            continue
        H, W = m.format.pixels_per_column, m.format.columns_per_frame
        image_cpy = image
        if image.shape != (H, W):
            click.secho(f"mask image doesn't match the size ({W}, {H}) "
                        f"for sensor[{i}], will scale", fg="yellow")
            # nearest neighbor keeps mask values exact
            rows = ((2 * np.arange(H) + 1) * image.shape[0]) // (2 * H)
            cols = ((2 * np.arange(W) + 1) * image.shape[1]) // (2 * W)
            image_cpy = image[rows[:, None], cols]

        # mask should always be in the range value of {0, 1} the mask operator is applied
        # to the raw frames, so we need to stagger the input image correctly per input sensor
        mask = image_cpy / 255.0
        mask = destagger(m, mask, inverse=True)
        masks.append(mask)

    field_list = None if fields is None else fields.strip().split(',')
    frame_set_iter = ctx.frame_set_iter

    def mask_iter():
        for frames in frame_set_iter():
            out = copy.deepcopy(frames)
            for idx, frame in enumerate(out):
                if frame and masks[idx] is not None:
                    so.mask(frame, field_list, masks[idx])
            yield out

    ctx.frame_set_iter = mask_iter  # type: ignore


def write_extrinsics_json(sensor_info_list: Iterable[Any], filename: str,
                          overwrite: bool = False) -> None:
    """Write per-sensor ``sensor_to_body`` transforms to an extrinsics json file.

    Raises ``FileExistsError`` if ``filename`` exists and ``overwrite`` is False.
    """
    import json
    from .source_save import _file_exists_error
    from ouster.sdk.core import Pose

    if os.path.exists(filename) and not overwrite:
        raise FileExistsError(_file_exists_error(filename))

    data: Dict[str, Any] = {}
    data["transforms"] = []
    for si in sensor_info_list:
        sensor = {}
        sensor["destination_frame"] = "world"
        sensor["source_frame"] = str(si.sn)
        pose = Pose(si.sensor_to_body)
        sensor["p_x"] = pose.position[0]
        sensor["p_y"] = pose.position[1]
        sensor["p_z"] = pose.position[2]
        sensor["q_w"] = pose.rotation[0]
        sensor["q_x"] = pose.rotation[1]
        sensor["q_y"] = pose.rotation[2]
        sensor["q_z"] = pose.rotation[3]
        data["transforms"].append(sensor)

    with open(filename, "w") as f:
        f.write(json.dumps(data))


@click.command
@click.argument("filename", required=True)
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.pass_context
@source_multicommand(type=SourceCommandType.SINGLE_CONSUMER)
def source_save_extrinsics(ctx: SourceCommandContext, filename: str, overwrite: bool, **kwargs) -> None:
    """Save the sensor to body transforms for each sensor to an extrinsics json file."""
    assert ctx.frame_set_source is not None

    try:
        write_extrinsics_json(ctx.frame_set_source.sensor_info, filename, overwrite)
    except FileExistsError as e:
        print(str(e))
        exit(1)


@click.command
@click.argument("filename", required=True)
@click.option('--filter', 'zone_files', default='both',
              type=click.Choice(['stl', 'zrb', 'both'], case_sensitive=False),
              help="Which zone files to save alongside the metadata. "
              "'stl' saves only each zone's STL mesh, 'zrb' saves only each zone's rendered ZRB, "
              "and 'both' saves both.")
@click.option('--output-type', default='zip',
              type=click.Choice(['zip', 'directory'], case_sensitive=False),
              help="Output file type. "
              "If 'zip', each zone set is saved as a zip file named FILENAME suffixed with the sensor serial number "
              "and sensor index, or as <serial number>_<sensor index>.zip inside FILENAME if it is an existing "
              "directory. "
              "If 'directory', each zone set is saved to a FILENAME/<serial number>_<sensor index> subdirectory.")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.pass_context
@source_multicommand(type=SourceCommandType.SINGLE_CONSUMER)
def source_save_zone_set(ctx: SourceCommandContext,
                         filename: str, zone_files: str,
                         output_type: str, overwrite: bool, **kwargs) -> None:
    """Save the zone set for each sensor to a zip file or directory.

    With zip output, the sensor serial number and the sensor's index in the source are appended
    to FILENAME, e.g. zones.zip becomes zones_122247000785_0.zip for the first sensor. If FILENAME is
    an existing directory, e.g. '.', the zips are saved inside it as <serial number>_<sensor index>.zip.
    With directory output, each zone set is written to a subdirectory of FILENAME named
    <serial number>_<sensor index>.
    """
    from .source_save import _file_exists_error
    assert ctx.frame_set_source is not None

    zone_sets = []
    for idx, si in enumerate(ctx.frame_set_source.sensor_info):
        if not isinstance(si, core.LidarInfo):
            continue
        if si.zone_set is None:
            print(f"Sensor idx {idx} sn {si.sn} has no zone set, skipping.")
            continue
        zone_sets.append((idx, si.sn, si.zone_set))

    if not zone_sets:
        print("No zone sets found in source.")
        exit(1)

    zone_set_filter = ZoneSetOutputFilter.STL_AND_ZRB
    if zone_files.lower() == 'stl':
        zone_set_filter = ZoneSetOutputFilter.STL
    elif zone_files.lower() == 'zrb':
        zone_set_filter = ZoneSetOutputFilter.ZRB

    save_as_zip = output_type.lower() == 'zip'
    if save_as_zip and os.path.isdir(filename):
        outputs = [(os.path.join(filename, f'{sn}_{idx}.zip'), idx, sn, zone_set)
                   for idx, sn, zone_set in zone_sets]
    elif save_as_zip:
        base, ext = os.path.splitext(filename)
        outputs = [(f"{base}_{sn}_{idx}{ext}", idx, sn, zone_set) for idx, sn, zone_set in zone_sets]
    else:
        if os.path.isfile(filename):
            print(f"Error: '{filename}' already exists and is a file. "
                  "Use --output-type=zip to save as a zip file, or remove the file and try again.")
            exit(1)
        outputs = [(os.path.join(filename, f'{sn}_{idx}'), idx, sn, zone_set) for idx, sn, zone_set in zone_sets]

    # check every output before writing any, so a conflict doesn't leave partial output behind
    for path, _, _, _ in outputs:
        if save_as_zip and os.path.isdir(path):
            print(f"Error: '{path}' already exists and is a directory. "
                  "Use --output-type=directory to save as a directory, or remove the directory and try again.")
            exit(1)
        if not save_as_zip and os.path.isfile(path):
            print(f"Error: '{path}' already exists and is a file. "
                  "Use --output-type=zip to save as a zip file, or remove the file and try again.")
            exit(1)
        if os.path.exists(path) and not overwrite:
            print(_file_exists_error(path))
            exit(1)

    try:
        if not save_as_zip:
            # ZoneSet.save_to_directory can't create missing parent directories
            os.makedirs(filename, exist_ok=True)
        for path, idx, sn, zone_set in outputs:
            if save_as_zip:
                zone_set.save(path, zone_set_filter)
            else:
                if os.path.isdir(path):
                    _remove_zone_set_files(path)
                zone_set.save_to_directory(path, zone_set_filter)
            print(f"Saved zone set for sensor idx {idx} sn {sn} to {path}")
    except (RuntimeError, ValueError, OSError) as e:
        print(f"Error: {e}")
        exit(1)


def _remove_zone_set_files(directory: str) -> None:
    """Remove the files a previous ZoneSet.save_to_directory may have written to ``directory``,
    so that overwriting it with a differently filtered zone set doesn't leave stale files behind.
    Other files are left in place."""
    for name in os.listdir(directory):
        path = os.path.join(directory, name)
        if os.path.isfile(path) and (name == 'metadata.json' or name.lower().endswith(('.stl', '.zrb'))):
            os.remove(path)


@click.command
@click.pass_context
@click.argument("filename", required=False)
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED)
def url_source_download(ctx: SourceCommandContext, filename: Optional[str], overwrite: bool) -> None:
    """
    Downloads the file at the given Ouster Studio Web or other URL.
    Make sure to properly escape any commas as they are separators in the CLI.
    """
    import requests
    import urllib.parse
    from .source_save import _file_exists_error
    from ouster.sdk.open_source import _url_to_osf_url

    if len(ctx.source_uris) > 1 and filename is not None:
        click.secho("Cannot provide filename for multiple downloads.", fg="red")
        exit(2)

    for source_url in ctx.source_uris:
        source_url = _url_to_osf_url(source_url)

        if filename is None:
            # if no filename provided, try grabbing it from the URL and decoding it
            raw_path = source_url.split("?")[0].split("/")[-1]
            raw_path = urllib.parse.unquote(raw_path, encoding='utf-8', errors='replace')
            out_filename = os.path.basename(raw_path)  # get rid of any dirs in case they were encoded
            if len(out_filename) == 0:
                click.secho("No filename found in URL. Please provide one.", fg="red")
                exit(2)
        else:
            out_filename = filename

        if os.path.exists(out_filename) and not overwrite:
            click.secho(_file_exists_error(out_filename), fg="red")
            exit(2)

        with open(out_filename, "wb") as f:
            print("Downloading to %s" % out_filename)
            response = requests.get(source_url, stream=True)
            total_length = response.headers.get('content-length')
            if total_length is None:  # no content length header
                f.write(response.content)
            else:
                dl = 0
                total_length_int = int(total_length)
                with ProgressBar(total_length_int // 1000, unit="kB", alpha=0.01) as bar:
                    for data in response.iter_content(chunk_size=500000):
                        dl += len(data)
                        bar.update(dl // 1000)
                        f.write(data)


@click.command
@click.option(
    '-c', '--config', default=None, type=click.Path(),
    help="Path to zone configuration zip to use. If not provided uses configuration"
    " from the underlying source."
)
@click.option('-l', '--live', default=None, type=str,
              help="Comma separated list of zones to make live."
                   " Cannot be combined with '--keep-live-ids'.")
@click.option('--no-render', is_flag=True, default=False,
              help="Do not render zones and use those stored in configuration.")
@click.option('--keep-sensor-to-body', '-s', is_flag=True, default=False,
              help="Use sensor to body transform from the zones stored in the"
                   " original source.")
@click.option('--keep-live-ids', '-z', is_flag=True, default=False,
              help="Use live zones ids from zone data in the original source. Cannot be combined with '--live'.")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR)
def source_emulate_zones(
    ctx: SourceCommandContext,
    config: Optional[str],
    live: Optional[str],
    no_render: bool,
    keep_sensor_to_body: bool,
    keep_live_ids: bool
):
    """
    Emulate zone monitoring off-sensor.
    """
    assert ctx.frame_set_source is not None
    assert ctx.frame_set_iter is not None
    frame_set_iter = ctx.frame_set_iter

    # TODO[tws] figure out what to do with Multi
    from ouster.sdk.core import EmulatedZoneMon, ZoneSet, ZONE_OCCUPANCY_FIELDNAME, \
        ZONE_STATES_FIELDNAME
    # Zone monitoring is a lidar feature; use the first lidar sensor.
    sensor_info = [info for info in ctx.frame_set_source.sensor_info
                   if isinstance(info, core.LidarInfo)][0]

    # TODO[tws] also support a directory
    zone_set: Optional[ZoneSet]
    if config is not None:
        zone_set = ZoneSet(config)
    else:
        zone_set = sensor_info.zone_set

    if not zone_set:
        print("emulate_zones: No zone monitor configuration available.")
        exit(1)

    if keep_sensor_to_body:
        if sensor_info.zone_set is None:
            print("emulate_zones: --keep-sensor-to-body requires existing zones.")
            exit(1)
        zone_set.sensor_to_body_transform = sensor_info.zone_set.sensor_to_body_transform

    # setting live zones by parameter
    if live is not None:
        if keep_live_ids:
            print("emulate_zones: --live and --keep-live-ids cannot be combined.")
            exit(1)
        live_zones = []
        for id in live.split(','):
            try:
                live_zones.append(int(id))
            except ValueError:
                raise click.exceptions.BadParameter("List of live zones must be a comma separated list of integers")
        zone_set.power_on_live_ids = live_zones

    # set a port and dest on the zm output if there isn't one
    if sensor_info.config.udp_port_zm is None:
        sensor_info.config.udp_port_zm = 7504
    if sensor_info.config.udp_dest_zm is None:
        sensor_info.config.udp_dest_zm = "127.0.0.1"
    sensor_info.format.zone_monitoring_enabled = True

    if keep_live_ids and sensor_info.zone_set:
        if len(sensor_info.zone_set.zones) > len(zone_set.zones):
            print("emulate_zones: --keep-live-ids requires new zone set to contain"
                  " as least as many zones as the previous.")
            exit(1)

    # render zones
    sensor_info.zone_set = zone_set
    if not no_render:
        print("Rendering zones....")
        zone_set.render(sensor_info)
        print("Finished rendering zones.")

    # update the source's configuration, which the viz uses for zone geometry
    sensor_info.zone_set = zone_set
    ctx.emulated_zone_monitoring_configuration = zone_set
    emulator = EmulatedZoneMon(ctx.emulated_zone_monitoring_configuration)

    def emulate_zones_iter() -> Iterable[List[Optional[LidarFrame]]]:
        for frames in frame_set_iter():  # type: ignore
            # TODO[tws] figure out what to do with Multi
            frame = frames[0]
            if frame.has_field(ZONE_STATES_FIELDNAME):
                if keep_live_ids:
                    data = frame.field(ZONE_STATES_FIELDNAME)
                    live_zones = []
                    for zone in data:
                        if zone.live != 0:
                            live_zones.append(zone.id)
                    emulator.set_live_zones(live_zones)
                frame.del_field(ZONE_STATES_FIELDNAME)
            if frame.has_field(ZONE_OCCUPANCY_FIELDNAME):
                frame.del_field(ZONE_OCCUPANCY_FIELDNAME)
            if frame.has_field("ZONE_PACKET_TIMESTAMP"):
                frame.del_field("ZONE_PACKET_TIMESTAMP")
            frame.add_field(ZONE_OCCUPANCY_FIELDNAME, np.uint16)
            emulator.calc_triggers(frame.field(ChanField.RANGE), frame.field(ZONE_OCCUPANCY_FIELDNAME))
            zone_monitor_update = emulator.get_packet()
            frame.add_field(ZONE_STATES_FIELDNAME, zone_monitor_update, FieldClass.FRAME_FIELD)
            ts = frame.get_last_valid_receive_timestamp()
            frame.add_field("ZONE_PACKET_TIMESTAMP", np.array([ts], np.uint64), FieldClass.FRAME_FIELD)
            yield frames

    ctx.frame_set_iter = emulate_zones_iter  # type: ignore


class SourceMultiCommand(click.MultiCommand):
    """This class implements the ouster-cli source command group.  It uses the
    `io_type` method to determine the source type and map it to the
    available sub commands for that type.

    The source is also added to the click context so that sub commands that use
    @click.pass_context have access to it."""

    commands: MutableMapping[Any, MutableMapping[str, click.Command]]

    def __init__(self, *args, **kwargs):
        kwargs['no_args_is_help'] = True

        super().__init__(*args, **kwargs)
        self.commands = {
            'ANY': {
                'align': source_align,
                'clip': source_clip,
                'emulate_zones': source_emulate_zones,
                'filter': source_filter,
                'ground': source_ground,
                'mask': source_mask,
                'normals': source_normals,
                'plumb': source_plumb,
                'reduce': source_reduce,
                'save': SourceSaveCommand('save',
                                          context_settings=dict(ignore_unknown_options=True,
                                                                allow_extra_args=True)),
                'save_extrinsics': source_save_extrinsics,
                'save_zone_set': source_save_zone_set,
                'select': source_select,
                'slice': source_slice,
                'stats': source_stats,
                'viz': source_viz,
                'walls': source_walls,
            },
            OusterIoType.SENSOR: {
                'save_raw': source_save_raw,
                # other sensor specific things are added in source_sensor.py
            },
            OusterIoType.PCAP: {
                'info': pcap_cli.pcap_info,
                'sensor_info': default_source_sensor_info,
                'save_raw': source_save_raw,
            },
            OusterIoType.OSF: {
                'dump': osf_cli.osf_dump,
                'info': osf_cli.osf_info,
                'parse': osf_cli.osf_parse,
                'pose_optimize': source_pose_optimize,
                'sensor_info': default_source_sensor_info,
            },
            OusterIoType.URL: {
                'info': osf_cli.osf_info,
                'sensor_info': default_source_sensor_info,
                'download': url_source_download,
                'parse': osf_cli.osf_parse,
                'dump': osf_cli.osf_dump,
            },
            OusterIoType.BAG: {
                'info': bag_info,
                'sensor_info': default_source_sensor_info,
                'save_raw': source_save_raw
            },
            OusterIoType.MCAP: {
                'info': bag_info,
                'sensor_info': default_source_sensor_info,
                'save_raw': source_save_raw
            },
            OusterIoType.PCD: {
                'save': model_save,
                'viz': model_viz,
            },
            OusterIoType.PLY: {
                'save': model_save,
                'viz': model_viz,
            },
            OusterIoType.LAS: {
                'save': model_save,
                'viz': model_viz
            },
            OusterIoType.LAZ: {
                'save': model_save,
                'viz': model_viz
            },
            OusterIoType.STL: {
                'viz': model_viz,
            }
        }
        self.non_frame_set_sources = [OusterIoType.PLY, OusterIoType.PCD, OusterIoType.STL,
                                      OusterIoType.LAS, OusterIoType.LAZ]

    def get_supported_source_types(self):
        return [iotype for iotype in self.commands.keys() if isinstance(iotype, OusterIoType)]

    def get_source_file_extension_str(self):
        exts = sorted(
            [extension_from_io_type(src_type)
                for src_type in self.commands.keys()
                if isinstance(src_type, OusterIoType) and extension_from_io_type(src_type)]
        )
        return _join_with_conjunction(exts)

    def format_commands(self, ctx, formatter):
        cmds = self.list_commands(ctx)
        sections = {}
        max_name_len = 0
        for name, commands in cmds.items():
            split = name.split(" ")
            section = split[-1] + " "
            if isinstance(commands, dict):
                for item in commands:
                    item = commands[item]
                    break
            else:
                item = commands
                section = ""
            if section not in sections:
                sections[section] = []
            sections[section].append((split[0], item))
            max_name_len = max(max_name_len, len(split[0]))

        for section, commands in sections.items():
            commands = sorted(commands, key=lambda x: x[0])
            with formatter.section(section + "Commands"):
                rows = []
                for name, cmd in commands:
                    if cmd.hidden:
                        continue
                    # Set short help to be the first sentence of each help string
                    if cmd.short_help is None and cmd.help is not None:
                        split = cmd.help.strip().split(".")
                        cmd.short_help = split[0] + "."
                    name = name + " " * (max_name_len - len(name))
                    rows.append((name, cmd.get_short_help_str()))
                formatter.write_dl(rows)

    def list_commands(self, click_ctx: click.core.Context):
        """Get the source type from the click context
        and return the list of appropriate sub command names"""
        source = click_ctx.params.get(_source_arg_name)
        if source and click_ctx.params.get("glob"):
            source = libglob.glob(source)[0]
        else:
            source = source.split(',')[0] if source else None

        if not source and CliArgs().has_any_of(click_ctx.help_option_names):
            # Build a map from command name to command
            command_to_types: Dict[str, Dict[Any, click.core.Command]] = {}
            for src_type in self.commands.keys():
                for command_name in self.commands[src_type].keys():
                    if command_name not in command_to_types:
                        command_to_types[command_name] = {}
                    if src_type == "ANY":
                        for supported_src_type in self.get_supported_source_types():
                            if supported_src_type in self.non_frame_set_sources:
                                continue
                            command_to_types[command_name][supported_src_type] = self.commands[src_type][command_name]
                    else:
                        command_to_types[command_name][src_type] = self.commands[src_type][command_name]

            # Postfix command name with names of supported source types
            command_to_types_renamed = {}
            for key, value in command_to_types.items():
                postfix = _join_with_conjunction(
                    [t.name.upper() for t in value.keys()], separator="|", conjunction="")
                command_to_types_renamed[f"{key} {postfix}"] = value

            return command_to_types_renamed

        file_extensions_str = self.get_source_file_extension_str()
        if not source:
            param_decls = [_source_arg_name]
            param = click.core.Argument(param_decls=param_decls)
            raise click.exceptions.MissingParameter(
                None, click_ctx, param=param)
        try:
            src_type = io_type(source)
            if src_type in self.non_frame_set_sources:
                return {**self.commands[src_type]}
            return {**self.commands[src_type], **self.commands["ANY"]}
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
        source = click_ctx.params.get(_source_arg_name) or ""
        do_glob = click_ctx.params.get("glob")
        click_ctx.ensure_object(SourceCommandContext)
        ctx: SourceCommandContext = click_ctx.obj
        # add source to context so the command can access it

        if do_glob:
            source_list = libglob.glob(source.strip())
            string = ""
            for source in source_list:
                if len(string):
                    string = string + ","
                string = string + source
            ctx.source_uri = string
        else:
            ctx.source_uri = source

        ctx.source_uris = [url.strip() for url in ctx.source_uri.split(',') if url.strip()]

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


source: SourceMultiCommand


@cli.group(cls=SourceMultiCommand, chain=True)  # type: ignore[no-redef]
@click.argument(_source_arg_name, required=True)
@click.option('-g', '--glob', is_flag=True, default=False,
              help="If set, glob the source URI and if multiple files are found"
                   " play them one after another if compatible.")
# TODO[UN]: should we implement this as options instead of flag similar to `on_eof`?
@click.option('--loop', is_flag=True, default=False, hidden=True,
              help="Restart from the beginning when the end of the file is reached")
@click.option('-m', '--meta', required=False, type=click_ro_file, multiple=True,
              help="Metadata for PCAP or BAG, helpful if automatic metadata resolution"
                   " fails or no metadata is present in the bag file.")
@click.option('--sensor-idx', default=None, type=int, help="Retrieve data from only the sensor with this index.")
@click.option('-l', '--lidar-port', default=None, type=int, help="Destination port of lidar data")
@click.option('-i', '--imu-port', default=None, type=int, help="Destination port of IMU data")
@click.option('-x', '--do-not-reinitialize', is_flag=True, default=None,
              help="Do not reinitialize (by default it will reinitialize if needed)")
@click.option('-y', '--no-auto-udp-dest', is_flag=True, default=None,
              help="Do not automatically set udp_dest (by default it will auto set udp_dest)")
@click.option('-s', '--soft-id-check', is_flag=True, default=None,
              help="Continue parsing lidar packets even if init_id/sn doesn't match with metadata")  # noqa
@click.option('-t', '--timeout', default=None, type=float, help="Seconds to wait for data [default: 1.0]")
@click.option('-f', '--filter', is_flag=True, help="Drop frames with missing data")
@click.option('-e', '--extrinsics', type=str, required=False,
              help="Use this arg to adjust Lidar sensor extrinsics of the source."
                    "\nSupported formats:"
                    "\n\n-e extrinsics.json ; A json file containing a per sensor extrinsics"
                    "\n\n-e identity ; Use this to override any stored extrinsics with identity matrix"
                    "\n\n-e X,Y,Z,R,P,Y ; 'X Y Z' for position (meters), 'R P Y' represent euler angles (deg)"
                    "\n\n-e X,Y,Z,QX,QY,QZ,QW ; 'X Y Z' for position (meters), 'QX, QY, QZ, QW' represent a quaternion"
                    "\n\n-e n1,n2,..,n16 ; 16 float representing a 2D array in a row-major order"
                    "\n\n-e zone ; Use sensor to body transform stored in zones."
                    "\n\nIf more than one sensor is present in the source and this argument is used"
                    " then the same extrinsics will be applied to all sensors except when using"
                    " an extrinsics file.")
@click.option('-p', '--initial-pose', type=str, required=False,
              help="Use this arg to set the starting pose of the source relative to the map origin when using "
                    "SLAM or localization. SLAM assumes its world frame is level and takes the initial "
                    "gravity as world -Z; the IMU plumb checks the pose and warns if its gravity is "
                    "more than 10 degrees off, keeping the pose. Localization takes gravity from "
                    "plumbing."
                    "\nSupported formats:"
                    "\n\n-p X,Y,Z,R,P,Y ; 'X Y Z' for position (meters), 'R P Y' represent euler angles (deg)"
                    "\n\n-p X,Y,Z,QX,QY,QZ,QW ; 'X Y Z' for position (meters), 'QX, QY, QZ, QW' represent a quaternion"
                    "\n\n-p n1,n2,..,n16 ; 16 float representing a 2D array in a row-major order")
@click.option('--fields', default=None, type=str,
              help="A comma separated list of field names to retrieve from the source."
                   " If not specified all (OSF) or defaults based on the packet format"
                   " (PCAP, Sensor, BAG, MCAP) are loaded.")
@click.option('--allow-major-version-mismatch', is_flag=True, default=False,
              help="If set, attempt to read OSF files whose major schema version is newer"
                   " than this SDK supports instead of raising an error.")
@click.option('--no-progress', is_flag=True, default=False,
              help="If set, hide the progress bar.")
@click.option('--reuse-ports', is_flag=True, default=False,
              help="If true, allow other programs to bind to the same sockets."
                   " Must be set on each listening program.")
@click.option('--no-defaults', is_flag=True, default=False,
              help="If true, do not change sensor settings other than operating mode and UDP destinations to defaults.")
@click.option('--lidar-profile', default=None, type=str,
              help="Override the UDPProfileLidar in the source to this value. Data is still"
                   " deserialized using the original lidar profile value.")
@click.option('--time', is_flag=True, default=False,
              help="If set, playback multiple file sources by time rather than for comparison.")
@click.option('--rcvbuf', type=click.IntRange(0),
              help="Set the UDP receive buffer size in bytes."
                   " Leave default or set to 0 to determine automatically.")
def source(source, loop: bool, meta: Tuple[str, ...],
           lidar_port: int, imu_port: int,
           extrinsics: Optional[str], initial_pose: Optional[str],
           do_not_reinitialize: bool, no_auto_udp_dest: bool,
           soft_id_check: bool, timeout: int, filter: bool,
           fields: Optional[str], allow_major_version_mismatch: bool,
           sensor_idx: Optional[int], glob: bool, no_progress: bool, reuse_ports: bool,
           no_defaults: bool, lidar_profile: Optional[str], time: bool,
           rcvbuf: Optional[int]):
    """Run a command with the specified source (SENSOR, PCAP, OSF, URL, BAG, MCAP, PCD, LAS, LAZ, PLY or STL) as SOURCE.
    For example, a sensor source: ouster-cli source os1-992xxx.local viz.
    To connect to multiple sensors you can provide them as a comma separated list.
    For example: ouster-cli source 192.168.1.201,192.168.1.202 viz.
    Multiple files can similarly be compared by providing them as a comma separated
     list, or by using '--glob'.
    To play back multiple frame sources (e.g. OSF, PCAP...) as if they were one specify the '--time' argument.
    """
    # old ouster-mapping package may interfere the ouster-sdk package
    is_ouster_mapping_installed()
    global _viz_wants_cycle
    _viz_wants_cycle = loop


@source.result_callback()
@click.pass_context
def process_commands(click_ctx: click.core.Context, callbacks: Iterable[SourceCommandCallback],
                     source: str, loop: bool, sensor_idx: Optional[int],
                     meta: Optional[Tuple[str, ...]], lidar_port: int, imu_port: int,
                     extrinsics: Optional[str], initial_pose: Optional[str],
                     do_not_reinitialize: bool, no_auto_udp_dest: bool, allow_major_version_mismatch: bool,
                     soft_id_check: bool, timeout: int, filter: bool, fields: Optional[str],
                     glob: bool, no_progress: bool, reuse_ports: bool, no_defaults: bool,
                     lidar_profile: Optional[str], time: bool, rcvbuf: Optional[int]) -> None:
    """Process all commands in a SourceMultiCommand, using each command's callback"""

    callbacks = list(callbacks)
    ctx: SourceCommandContext = click_ctx.obj
    command_names = ctx.invoked_command_names
    resolved_extrinsics: Optional[Union[str, np.ndarray]] = None
    resolved_initial_pose: Optional[np.ndarray] = None

    if extrinsics and extrinsics != "zone":
        resolved_extrinsics = parse_extrinsics_from_string(extrinsics)

    if initial_pose:
        resolved_initial_pose = parse_extrinsics_from_string(initial_pose)
        if (not isinstance(resolved_initial_pose, np.ndarray) or
                resolved_initial_pose.shape != (4, 4)):
            raise click.exceptions.UsageError(f"'error processing initial_pose: {initial_pose},"
                                              " check help for proper usage")

    if not meta:
        meta = None

    field_names = None
    if fields is not None:
        field_names = fields.strip().split(',')

    # save some options for use later and for the open_source call
    ctx.source_options["meta"] = meta
    ctx.source_options["do_not_reinitialize"] = do_not_reinitialize
    ctx.source_options["no_auto_udp_dest"] = no_auto_udp_dest
    ctx.source_options["soft_id_check"] = soft_id_check
    ctx.source_options["lidar_port"] = lidar_port
    ctx.source_options["imu_port"] = imu_port
    ctx.source_options["timeout"] = timeout
    ctx.source_options["rcvbuf_size"] = rcvbuf
    if reuse_ports:
        ctx.source_options["reuse_ports"] = reuse_ports
    # negative or None sensor-index indicates all sensors
    if sensor_idx is not None and sensor_idx >= 0:
        ctx.source_options["sensor_idx"] = sensor_idx
    if resolved_extrinsics is not None:
        if isinstance(resolved_extrinsics, str):
            ctx.source_options["extrinsics_file"] = resolved_extrinsics
        else:
            ctx.source_options["extrinsics"] = [np.array(resolved_extrinsics).reshape((4, 4))]
    ctx.source_options["field_names"] = field_names
    if allow_major_version_mismatch:
        def error_handler(severity, msg):
            print(severity, msg)
        ctx.source_options["error_handler"] = error_handler

    # remove any that are just None
    set_list = {}
    for name in ctx.source_options:
        if ctx.source_options[name] is not None:
            set_list[name] = ctx.source_options[name]
    ctx.source_options = set_list

    if resolved_initial_pose is not None:
        ctx.other_options["initial_pose"] = resolved_initial_pose
    if lidar_profile is not None:
        ctx.other_options["lidar_profile"] = lidar_profile

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

    # Ensure that no other commands are present, if a MULTICOMMAND_UNSUPPORTED is present
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
    has_true_consumer = False
    for idx, c in enumerate(callbacks):
        if c.type in [SourceCommandType.PROCESSOR_UNREPEATABLE, SourceCommandType.PROCESSOR]:
            last_processor_idx = idx
            last_processor_name = command_names[idx]
        elif c.type in [SourceCommandType.CONSUMER, SourceCommandType.SINGLE_CONSUMER]:
            last_consumer_idx = idx
            last_consumer_name = command_names[idx]
            if c.type is SourceCommandType.CONSUMER:
                has_true_consumer = True

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
        source_list = ctx.source_uris

        is_sensor = io_type(source_list[0]) == OusterIoType.SENSOR

        # configure new imu format on 3.2 sensors
        if is_sensor and (not do_not_reinitialize) and (not no_defaults):
            configs = []
            for uri in source_list:
                config = core.LidarConfig()
                old_config = sensor.get_config(uri, active=True)
                if old_config.imu_packets_per_frame is not None:
                    if old_config.udp_profile_lidar != core.UDPProfileLidar.LEGACY and \
                        old_config.udp_profile_imu == core.UDPProfileIMU.LEGACY:
                        print("Will change udp_profile_imu to ACCEL32_GYRO32_NMEA."
                              " To disable this provide the --no-defaults argument.")
                        config.udp_profile_imu = core.UDPProfileIMU.ACCEL32_GYRO32_NMEA
                    if old_config.accel_fsr != core.FullScaleRange.EXTENDED:
                        print("Will change accel_fsr to EXTENDED."
                              " To disable this provide the --no-defaults argument.")
                        config.accel_fsr = core.FullScaleRange.EXTENDED
                    if old_config.gyro_fsr != core.FullScaleRange.EXTENDED:
                        print("Will change gyro_fsr to EXTENDED."
                              " To disable this provide the --no-defaults argument.")
                        config.gyro_fsr = core.FullScaleRange.EXTENDED
                configs.append(config)
            ctx.source_options["sensor_config"] = configs

        # Decide how to play back multiple sources if we get them

        try:
            frame_set_source: FrameSetSource
            if (not is_sensor) and len(source_list) > 1:
                # need to not set sensor_idx on the original sources for this case
                idx = cast(int, ctx.source_options.get("sensor_idx"))
                if idx is not None:
                    del ctx.source_options["sensor_idx"]
                l = []
                for src in source_list:
                    l.append(open_source(src, **ctx.source_options, collate=False))

                if not time:
                    from ouster.sdk.core.comparison_frame_set_source import ComparisonFrameSetSource
                    frame_set_source = ComparisonFrameSetSource(l)
                    if idx is not None:
                        raise click.exceptions.UsageError("--sensor-idx not supported with"
                                                          " comparison mode. Try using with"
                                                          " --time.")
                else:
                    frame_set_source = MultiFrameSetSource(l)
                    if idx is not None:
                        frame_set_source = frame_set_source.single(idx)
                    else:
                        frame_set_source = collate(frame_set_source)
            else:
                frame_set_source = open_source(source_list, **ctx.source_options, collate=True)
                if frame_set_source.contains_collations:
                    ctx.save_collations = True

            if "lidar_profile" in ctx.other_options:
                for info in frame_set_source.sensor_info:
                    if not isinstance(info, core.LidarInfo):
                        continue
                    info.format.udp_profile_lidar = ctx.other_options["lidar_profile"]
                    info.config.udp_profile_lidar = ctx.other_options["lidar_profile"]

            # override extrinsics from zones if requested (zones are lidar-only)
            if extrinsics == "zone":
                for info in frame_set_source.sensor_info:
                    if not isinstance(info, core.LidarInfo):
                        continue
                    if info.zone_set:
                        info.sensor_to_body = np.asarray(info.zone_set.sensor_to_body_transform)
                        info.clear_cache()
                    else:
                        click.secho("ERROR: cannot set extrinsics to zone as no zone_set present on sensor in source.",
                                fg="red")
                        return

            # HACK: We need to redesign how the FrameSetSource is passed is passed and
            # used, the current pipeline skips the frame_set_source and simply uses the
            # iterators which then results in this problem where any operation that
            # may affect the frame_set_source isn't taking effect such as the reduce
            # or select operations.
            if command_names[0] == "reduce":
                if "select" in command_names[1:]:
                    raise click.exceptions.UsageError(
                        "reduce needs to be the first command after source.")
                reject_camera_streams(frame_set_source, "reduce")
                beams_idx = sys.argv.index("reduce")
                beams = [int(sys.argv[beams_idx + 1])] * len(frame_set_source.sensor_info)
                from ouster.sdk.core import ReducedFrameSetSource
                ctx.frame_set_source = ReducedFrameSetSource(
                    frame_set_source, beams=beams)
            elif command_names[0] == "select":
                if "reduce" in command_names[1:]:
                    raise click.exceptions.UsageError(
                        "select needs to be the first command after source.")
                reject_camera_streams(frame_set_source, "select")
                indices_idx = sys.argv.index("select")
                indices = parse_beam_indices(sys.argv[indices_idx + 1])
                beam_indices = [indices] * len(frame_set_source.sensor_info)
                from ouster.sdk.core import SelectedFrameSetSource
                ctx.frame_set_source = SelectedFrameSetSource(
                    frame_set_source, indices=beam_indices)
            else:
                if "reduce" in command_names[1:]:
                    raise click.exceptions.UsageError(
                        "reduce needs to be the first command after source.")
                if "select" in command_names[1:]:
                    raise click.exceptions.UsageError(
                        "select needs to be the first command after source.")
                ctx.frame_set_source = frame_set_source

            # Fill in default UDP ports if missing (e.g., when loading from OSF)
            # This ensures downstream operations (replay, save, etc.) have valid ports
            DEFAULT_LIDAR_PORT = 7502
            DEFAULT_IMU_PORT = 7503
            for info in ctx.frame_set_source.sensor_info:
                if not isinstance(info, core.LidarInfo):
                    continue
                if info.config.udp_port_lidar is None:
                    info.config.udp_port_lidar = DEFAULT_LIDAR_PORT
                if info.config.udp_port_imu is None:
                    info.config.udp_port_imu = DEFAULT_IMU_PORT

        except SourceURLException as e:
            sub_exception = e.get_sub_exception()
            is_dupe_port = isinstance(sub_exception, PcapDuplicatePortException)
            if is_dupe_port and soft_id_check:
                click.secho("ERROR: --soft-id-check is not supported for multi-sensor datasets.",
                            fg="red")
                return
            else:
                raise

        slice_range = None
        slice_hint = None
        # speed up slicing on indexed OSF if slicing comes first
        global _last_slice
        if command_names[0] == "slice" and _last_slice:
            # we can only speed it up if it's indexed
            if ctx.frame_set_source.is_indexed:
                # at the moment we can only handle index based slices (where the start is not a float)
                # TODO: support time based slices
                if type(_last_slice[0]) is not float and type(_last_slice[1]) is not float:
                    # finally calculate wrap-around start and end indexes assuming cycle is set
                    # TODO: revisit when we revisit cycle/loop
                    slice_range = _last_slice
                    callbacks = callbacks[1:]  # remove the callback since we don't need it now
            else:
                if type(_last_slice[0]) is not float and type(_last_slice[1]) is not float:
                    slice_hint = _last_slice
            _last_slice = None  # globals are the root of all evil

        # print any timeout exceptions we get and lazily instantiate the frame
        # source iterator so consumers have a chance to set flags like loop
        def catch_iter():
            last_dropped = 0

            src = ctx.frame_set_source

            # if this is a sensor source drop the first of each frame to avoid an almost
            # guaranteed partial frame
            if is_sensor:
                next(iter(src))

            if slice_range:
                src = src[slice(slice_range[0], slice_range[1], slice_range[2])]

            total = operator.length_hint(src, 0)

            # adjust total based on slice for unindexed sources
            # note this ignores the increment factor and start because we can't skip on unindexed sources
            if slice_hint is not None and slice_hint[1] is not None:
                # fake the length for a sensor if we set an upper limit in the slice
                if is_sensor:
                    total = slice_hint[1]

                # length is just the min of the request or estimated size
                total = min(slice_hint[1], total)

            with ProgressBar(total, unit="frames") as bar:
                for idx, frame in enumerate(src):
                    if not no_progress:
                        bar.update(idx)
                    # drop incomplete frames
                    if filter:
                        for i in range(0, len(frame)):
                            profile = frame[i].sensor_info.format.udp_profile_lidar
                            if not frame[i].complete() and profile != core.UDPProfileLidar.OFF:
                                frame[i] = None
                        # skip rather than return empty array if somehow all were incomplete
                        all_none = True
                        for s in frame:
                            if s is not None:
                                all_none = False
                        if all_none:
                            continue
                    if hasattr(ctx.frame_set_source, "dropped_frames"):
                        dropped = ctx.frame_set_source.dropped_frames
                        if dropped > last_dropped:
                            click.echo(click.style(f"Warning: Dropped {dropped - last_dropped} lidar frames.",
                                                fg="yellow"))
                            last_dropped = dropped
                    yield frame
                return
        ctx.frame_set_iter = catch_iter  # type: ignore

        assert ctx.frame_set_source is not None

        # Execute multicommand callbacks

        # Dataflow between callbacks occur entirely through the frame iterator. Callbacks may additionally
        # register functions in ctx.thread_fns to be run in individual threads
        # Callback invariants:
        #   1. Must leave ctx.frame_set_iter in a consumable state after invocation.
        #       If a callback consumes the original ctx.frame_set_iter, it must set ctx.frame_set_iter
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
        #   2. map a processing function onto frames, such that processing is implicitly called while iterating
        #   3. set ctx.frame_set_iter to a new iterator, and register a processing thread in ctx.thread_fns
        #   4. create a CoupledTee from ctx.frame_set_iter, and re-set ctx.frame_set_iter to one of
        #       the resultant tees

        ctx.main_thread_fn = None
        ctx.terminate_evt = threading.Event()
        for c in callbacks:
            c.callback_fn(ctx)

        # Define a function to consume ctx.frame_set_iter
        def pipeline_flush():
            try:
                do_loop = _viz_wants_cycle or loop
                while True:
                    for frames in ctx.frame_set_iter():
                        if ctx.terminate_evt.is_set():
                            return
                        pass
                    # terminate event might get set on loop exit above
                    # make sure we don't process another frame just to exit
                    if ctx.terminate_evt.is_set() or not do_loop:
                        break
            except ClientTimeout as ex:
                click.echo(click.style(f"Error: {ex}", fg="red"))
                ctx.terminate_evt.set()
            except Exception as ex:
                # Terminate everything if we get an unhandled exception
                ctx.terminate_evt.set()
                raise ex

        # If we have any consumers, run the main thread and pipeline flush until end or ctrl-c
        if has_true_consumer:
            thread: Optional[threading.Thread] = None
            try:
                if ctx.main_thread_fn is not None:
                    # If there is a main thread run that on the main thread
                    thread = threading.Thread(target=pipeline_flush)
                    thread.start()
                    ctx.main_thread_fn()
                else:
                    # Otherwise just run pipeline flush on main thread
                    pipeline_flush()
            finally:
                # Make sure to always call terminate even if something throws
                ctx.terminate_evt.set()

                # Wait for threads to terminate
                if thread is not None:
                    # On Windows thread.join(None) blocks through KeyboardInterrupts
                    # Make sure that we only block temporarily so we can catch them
                    while True:
                        thread.join(0.1)
                        if not thread.is_alive():
                            break

        # TODO: https://ouster.atlassian.net/browse/FLEETSW-6470
        if hasattr(ctx.frame_set_source, "id_error_count") and ctx.frame_set_source.id_error_count > 0:
            print(f"WARNING: {ctx.frame_set_source.id_error_count} packets with "
                  f"mismatched init_id/sn were detected.")
            if not soft_id_check:
                print("NOTE: To disable strict init_id/sn checking use "
                      "--soft-id-check option (may lead to parsing "
                      "errors)")
        if hasattr(ctx.frame_set_source, "size_error_count") and ctx.frame_set_source.size_error_count > 0:
            print(f"WARNING: {ctx.frame_set_source.size_error_count} packets with unexpected"
                  f" size detected and discarded. You may have the incorrect udp_profile_lidar in your metadata.")
        # Do not close the frame source manually here. Memory will be freed by python/GC on program exit.
