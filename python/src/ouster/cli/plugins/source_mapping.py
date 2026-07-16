from typing import cast

import click
import logging
import numpy as np
from functools import partial
from ouster.sdk.core import FrameSetSource
from ouster.cli.plugins.source import source  # type: ignore
from ouster.sdk.core import first_valid_column_pose
from ouster.cli.plugins.source_util import (source_multicommand,
                                            SourceCommandType,
                                            SourceCommandContext)
from ouster.sdk.util.extrinsics import rotation_matrix_to_quaternion     # type: ignore
from ouster.sdk.mapping import SlamConfig, SlamEngine
import ouster.sdk._bindings.mapping as mapping_bindings


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('mapping')


def auto_loop_cell_size_m(po: mapping_bindings.PoseOptimizer) -> float:
    """Compute loop cell size from trajectory XY span.

    Returns cell_size_m. Formula:
      cell_size_m = 7 + 0.004 * max(span_x, span_y)
    """
    base_cell_size_m = 7.0
    try:
        poses = po.get_poses(mapping_bindings.SamplingMode.KEY_FRAMES)
    except Exception:
        return base_cell_size_m

    poses_np = np.asarray(poses, dtype=np.float64)
    if poses_np.ndim != 3 or poses_np.shape[0] == 0 or poses_np.shape[1:] != (4, 4):
        return base_cell_size_m

    positions = poses_np[:, :3, 3]
    if positions.size == 0:
        return base_cell_size_m

    mins = np.nanmin(positions, axis=0)
    maxs = np.nanmax(positions, axis=0)
    if not np.all(np.isfinite(mins)) or not np.all(np.isfinite(maxs)):
        return base_cell_size_m

    span_x = float(maxs[0] - mins[0])
    span_y = float(maxs[1] - mins[1])
    span_xy = max(span_x, span_y)
    if not np.isfinite(span_xy) or span_xy < 0.0:
        return base_cell_size_m

    return base_cell_size_m + 0.004 * span_xy


@click.command
@click.option('--max-range', required=False, show_default=True,
              default=150.0, help="Max valid range")
@click.option('--min-range', required=False, show_default=True,
              default=1.0, help="Min valid range")
@click.option('-v', '--voxel-size', required=False, default=None,
              type=float, show_default=False,
              help="Voxel map size (meters). If not provided, "
                   "auto generate. [default: (auto); x>0]")
@click.option('--max-iterations', required=False, show_default=True,
              default=500, type=int,
              help="Maximum number of ICP registration iterations")
@click.option('--deskew-method', type=click.Choice(['auto', 'none', 'constant_velocity', 'imu_deskew']),
              default='auto', show_default=True,
              help="Method used for motion compensation (deskewing) of point clouds")
@click.option('--viz-local-map', is_flag=True, default=False,
              help="When chained with viz, draw every point in the SLAM local map and update it every frame.")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_slam(ctx: SourceCommandContext, max_range: float, min_range: float,
                voxel_size: float, max_iterations: int,
                deskew_method: str, viz_local_map: bool) -> None:
    """
    Run SLAM with a SOURCE.\n

    Example values for voxel_size:\n
        Outdoor: 0.8 - 1.5\n
        Large indoor: 0.4 - 0.8\n
        Small indoor: 0.1 - 0.5\n
    If voxel_size is not specified, the algorithm will use the first lidar frame to calculate it.\n
    Small voxel size could give more accurate results but take more memory and
    longer processing. For real-time slam, considing using a slightly larger voxel size
    and use visualizer to monitor the SLAM process.
    """

    # validate inputs
    if voxel_size is not None and voxel_size <= 0:
        raise click.UsageError("Voxel size must be greater than 0")

    if min_range < 0 or max_range < 0:
        raise click.UsageError("min_range and max_range must not be a negative number")

    if max_iterations <= 0:
        raise click.UsageError("max_iterations must be greater than 0")

    if viz_local_map and "viz" not in ctx.invoked_command_names:
        raise click.UsageError("--viz-local-map requires chaining with viz")

    ctx.slam_local_map_viz_enabled = viz_local_map

    def make_kiss_slam() -> SlamEngine:
        config = SlamConfig.create("lio")
        config.deskew_method = deskew_method
        config.min_range = min_range
        config.max_range = max_range
        config.voxel_size = voxel_size if voxel_size is not None else 0.0
        config.max_iterations = max_iterations
        config.initial_pose = ctx.other_options.get("initial_pose", np.eye(4))

        return SlamEngine.create(cast(FrameSetSource, ctx.frame_set_source).sensor_info,
                          config)

    def slam_iter(frame_set_source):
        slam_engine = make_kiss_slam()
        local_map_frame_id = 0
        for frames in frame_set_source():
            updated_frames = slam_engine.update(frames)
            if ctx.slam_local_map_viz_enabled:
                points = slam_engine.get_point_cloud()
                ctx.slam_local_map_viz_points = (local_map_frame_id, points)
                local_map_frame_id += 1
            yield updated_frames

    ctx.frame_set_iter = partial(slam_iter, ctx.frame_set_iter)  # type: ignore


@click.command
@click.argument('filename', required=True, type=str)
@click.option('-n', '--sensor-idx', type=int, default=0, show_default=True,
              help="Select specific sensor based on index within the file to save")
@click.option('--tum', is_flag=True, default=False,
              help="Save the trajectory in TUM format. Default is False which is CSV format")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def save_trajectory(ctx: SourceCommandContext, filename: str, sensor_idx: int, tum: bool) -> None:
    """
    Save a trajectory of the movement of selected sensor to a file
    """

    saved = False

    def trajectory_dump(frame_set_source):
        # only save during the first loop
        nonlocal saved
        if not saved:
            saved = True
        else:
            for frames in frame_set_source():
                yield frames
            return

        with open(filename, "wt") as f:
            if tum:
                # TUM format
                f.write("# timestamp tx ty tz qx qy qz qw\n")
            else:
                # CSV format
                f.write("timestamp,x,y,z,qx,qy,qz,qw\n")

            for frames in frame_set_source():
                frame = frames[sensor_idx]
                if frame is None:
                    continue
                try:
                    frame_ts = frame.timestamp[frame.get_first_valid_column()]
                    frame_pose = first_valid_column_pose(frame)
                except RuntimeError:
                    continue
                p = frame_pose[:3, 3]
                r = rotation_matrix_to_quaternion(frame_pose[:3, :3])

                if tum:
                    # frame_ts is in nanoseconds; convert to seconds for TUM/evo.
                    frame_ts_s = float(frame_ts) * 1e-9
                    f.write(f"{frame_ts_s:.9f} {p[0]} {p[1]} {p[2]} {r[1]} {r[2]} {r[3]} {r[0]}\n")
                else:
                    f.write(f"{frame_ts},{p[0]},{p[1]},{p[2]},{r[1]},{r[2]},{r[3]},{r[0]}\n")
                yield frames

    # address generator type later
    print(f"Saving the trajectory to {filename} ...")
    ctx.frame_set_iter = partial(trajectory_dump, ctx.frame_set_iter)  # type: ignore


source.commands['ANY']['slam'] = source_slam
source.commands['ANY']['save_trajectory'] = save_trajectory
