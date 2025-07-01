from typing import cast, List, Optional
import os
import sys
import click
import logging
import laspy
import numpy as np
from functools import partial
from ouster.sdk.core import OusterIoType, ScanSource, LidarScan, voxel_downsample
from ouster.cli.plugins.source import source  # type: ignore
from ouster.sdk.util import default_scan_fields
from ouster.cli.plugins.source_save import (SourceSaveCommand,
                                            determine_filename,
                                            create_directories_if_missing,
                                            _file_exists_error)
from ouster.sdk.core import (ChanField,
                             XYZLut,
                             first_valid_column_pose,
                             dewarp)
from ouster.cli.plugins.source_util import (source_multicommand,
                                            SourceCommandType,
                                            SourceCommandContext)
from ouster.sdk.util.extrinsics import rotation_matrix_to_quaternion     # type: ignore


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('mapping')


def save_pointcloud(filename: str, cloud: np.ndarray, ascii: bool = False, field="unknown"):
    if filename.endswith(".ply"):
        # CloudCompare PLY color point cloud using the range 0-1
        with open(filename, 'wb') as f:
            # Write PLY header
            f.write("ply\n".encode('utf-8'))
            if ascii:
                f.write("format ascii 1.0\n".encode('utf-8'))
            else:
                f.write("format binary_little_endian 1.0\n".encode('utf-8'))
            f.write("element vertex {}\n".format(cloud.shape[0]).encode('utf-8'))
            f.write("property float x\n".encode('utf-8'))
            f.write("property float y\n".encode('utf-8'))
            f.write("property float z\n".encode('utf-8'))
            if cloud.shape[1] == 4:
                f.write(f"property float {field}\n".encode('utf-8'))
            f.write("end_header\n".encode('utf-8'))
            if ascii:
                if cloud.shape[1] == 4:
                    for i in range(cloud.shape[0]):
                        f.write("{} {} {} {}\n".format(
                                    cloud[i, 0], cloud[i, 1],
                                    cloud[i, 2], cloud[i, 3] / 255).encode('utf-8'))
                else:
                    for i in range(cloud.shape[0]):
                        f.write("{} {} {}\n".format(
                                    cloud[i, 0], cloud[i, 1],
                                    cloud[i, 2]).encode('utf-8'))
            else:
                # todo scale intensity?
                bytes = cloud.astype(np.float32).tobytes()
                f.write(bytes)
    elif filename.endswith(".pcd"):
        # write as binary pcd
        with open(filename, 'wb') as f:
            # Write PCD header
            if cloud.shape[1] == 4:
                f.write(f"FIELDS x y z {field}\n".encode('utf-8'))
                f.write("SIZE 4 4 4 4\n".encode('utf-8'))
                f.write("TYPE F F F F\n".encode('utf-8'))
                f.write("COUNT 1 1 1 1\n".encode('utf-8'))
            else:
                f.write("FIELDS x y z\n".encode('utf-8'))
                f.write("SIZE 4 4 4\n".encode('utf-8'))
                f.write("TYPE F F F\n".encode('utf-8'))
                f.write("COUNT 1 1 1\n".encode('utf-8'))
            f.write(f"WIDTH {cloud.shape[0]}\n".encode('utf-8'))
            f.write("HEIGHT 1\n".encode('utf-8'))
            f.write(f"POINTS {cloud.shape[0]}\n".encode('utf-8'))
            if ascii:
                f.write("DATA ascii\n".encode('utf-8'))
                if cloud.shape[1] == 4:
                    for i in range(cloud.shape[0]):
                        f.write("{} {} {} {}\n".format(
                                    cloud[i, 0], cloud[i, 1],
                                    cloud[i, 2], cloud[i, 3]).encode('utf-8'))
                else:
                    for i in range(cloud.shape[0]):
                        f.write("{} {} {}\n".format(
                                    cloud[i, 0], cloud[i, 1],
                                    cloud[i, 2]).encode('utf-8'))
            else:
                f.write("DATA binary\n".encode('utf-8'))
                bytes = cloud.astype(np.float32).tobytes()
                f.write(bytes)


@click.command
@click.option('--max-range', required=False, show_default=True,
              default=150.0, help="Max valid range")
@click.option('--min-range', required=False, show_default=True,
              default=1.0, help="Min valid range")
@click.option('-v', '--voxel-size', required=False,
              type=float, help="Voxel map size (meters)")
@click.option('-d', '--dump-map', required=False,
              default="", type=str, help="Dumps the map to a ply file")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_slam(ctx: SourceCommandContext, max_range: float, min_range: float,
                voxel_size: float, dump_map: str) -> None:
    """
    Run SLAM with a SOURCE.\n

    Example values for voxel_size:\n
        Outdoor: 0.8 - 1.5\n
        Large indoor: 0.4 - 0.8\n
        Small indoor: 0.1 - 0.5\n
    If voxel_size is not specified, the algorithm will use the first lidar scan to calculate it.\n
    Small voxel size could give more accurate results but take more memory and
    longer processing. For real-time slam, considing using a slightly larger voxel size
    and use visualizer to monitor the SLAM process.
    """

    # validate inputs
    if voxel_size is not None and voxel_size <= 0:
        raise click.UsageError("Voxel size must be greater than 0")

    if min_range < 0 or max_range < 0:
        raise click.UsageError("min_range and max_range must not be a negative number")

    if dump_map:
        if not dump_map.endswith(".ply") and not dump_map.endswith(".pcd"):
            raise click.UsageError("--dump-map must be be in .ply or .pcd format")
    try:
        from ouster.sdk.mapping import SlamConfig, SlamEngine
        from ouster.sdk.mapping.util import determine_voxel_size
    except ImportError as e:
        raise click.ClickException(click.style("kiss-icp, a package required for slam, is "
                                    f"unsupported on this platform. Error: {str(e)}", fg='red'))

    def make_kiss_slam() -> SlamEngine:

        def live_sensor_voxel_size(scans: List[Optional[LidarScan]]) -> Optional[float]:
            """a customized version of determine_voxel_size for live sensors"""
            voxel_size = determine_voxel_size(scans)
            if voxel_size is not None:
                if cast(ScanSource, ctx.scan_source).is_live:
                    logger.info("Choosing a larger voxel size to support real-time processing of live sensors.")
                    voxel_size *= 2.2 * voxel_size
                logger.info(f"voxel-size arg is not set, using an estimated value of {voxel_size:.4g} m.")
            return voxel_size

        config = SlamConfig()
        config.min_range = min_range
        config.max_range = max_range
        config.voxel_size = live_sensor_voxel_size if voxel_size is None else voxel_size
        config.initial_pose = ctx.other_options["initial_pose"]
        config.backend = "kiss"

        return SlamEngine(infos=cast(ScanSource, ctx.scan_source).sensor_info,
                          config=config)

    def slam_iter(scan_source):
        slam_engine = make_kiss_slam()
        for scans in scan_source():
            scan = scans[0]
            if scan is None:
                continue

            yield slam_engine.update(scans)

        # only dump the map once
        nonlocal dump_map
        if dump_map:
            points = slam_engine._backend.ouster_kiss_icp.local_map.point_cloud()
            save_pointcloud(dump_map, points)
            logger.info(f"map was dumped to {dump_map}")
            dump_map = False

    ctx.scan_iter = partial(slam_iter, ctx.scan_iter)  # type: ignore


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('-v', '--voxel-size', default=0.1, help="Voxel map size for downsampling"
              "This parameter is the same as the open3D voxel size. Default value is 0.1. "
              "The bigger the value, the fewer points it outputs")
@click.option('--field',
              required=False,
              type=click.Choice(['SIGNAL',
                                 'NEAR_IR',
                                 'REFLECTIVITY'],
                                case_sensitive=False),
              default="REFLECTIVITY",
              help="Chanfield for output file key value. Choose between SIGNAL, NEAR_IR, "
              "REFLECTIVITY. Default field is REFLECTIVITY")
@click.option('--decimate', required=False, type=bool,
              default=True, help="Downsample the point cloud to output. Default is On")
@click.option('--verbose', is_flag=True, default=False,
              help="Print point cloud status much frequently. Default is Off")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite "
              "existing files with the same name")
@click.option('--max-z', default=None, type=float, help="max z threshold for point cloud saving")
@click.option('--min-z', default=None, type=float, help="min z threshold for point cloud saving")
@click.option('-f', '--pts-per-file', default=100000000, type=int,
              help="the number of points per output file. Default is 100000000")
@click.option('--ascii', is_flag=True, default=False,
              help="Output files in ASCII rather than binary format")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def point_cloud_convert(ctx: SourceCommandContext, filename: str, prefix: str,
                        dir: str, voxel_size: float, field: str, decimate: bool,
                        overwrite: bool, verbose: bool, max_z: float,
                        min_z: float, pts_per_file: int, ascii: bool, **kwargs) -> None:

    scans_iter = ctx.scan_iter
    infos = ctx.scan_source.metadata  # type: ignore

    outfile_ext = "." + kwargs["format"]
    output_file_path = determine_filename(prefix, dir, filename,
                                          outfile_ext, infos[0])
    create_directories_if_missing(output_file_path)

    # files pre exist check step
    file_wo_ext, outfile_ext = os.path.splitext(output_file_path)
    may_existed_file = file_wo_ext + '-000' + outfile_ext

    if (os.path.isfile(output_file_path) or os.path.isfile(may_existed_file)) and not overwrite:
        print(_file_exists_error(f'{output_file_path} or {may_existed_file}'))
        exit(1)

    already_saved = False
    empty_pose = True

    def convert_iter():
        points_to_process = np.empty(shape=[0, 3])
        keys_to_process = np.empty(shape=[0, 1])
        points_keys_total = np.empty(shape=[0, 4])

        # hard-coded parameters and counters #
        # affect the running time. smaller value mean longer running time. Too large
        # may lead to crash
        down_sample_steps = 100
        # affect per output file size. makes output file size ~1G
        file_numb = 0

        # variables for point cloud status printout
        points_sum = 0
        points_saved = 0
        points_out_range = 0
        points_down_removed = 0

        valid_fields = default_scan_fields(
            infos[0].format.udp_profile_lidar)

        found = False
        field_names = []
        for f in valid_fields:
            if f.name == field:
                found = True
            field_names.append(f.name)

        if not found:
            valid_fields_str = ", ".join(field_names)

            sys.exit(f"Exit! field {field} is not available in the low bandwidth mode\n"
                     f"use -f and choose a valid field from {valid_fields_str}")

        xyzlut_list = []
        for info in infos:
            xyzlut_list.append(XYZLut(info, use_extrinsics=True))

        def process_points(points, keys):
            nonlocal points_keys_total, points_down_removed, points_saved
            pts_size_before = points.shape[0]
            z_min_filter = np.ones(points.shape[0], dtype=bool)  # Initialize to all True
            z_max_filter = np.ones(points.shape[0], dtype=bool)
            if min_z is not None:
                z_min_filter = points[:, 2] >= min_z

            if max_z is not None:
                z_max_filter = points[:, 2] <= max_z

            # Apply range filter if any of min_z or max_z is specified
            if min_z is not None or max_z is not None:
                z_range_filter = z_min_filter & z_max_filter
                points = points[z_range_filter]
                keys = keys[z_range_filter]

            if decimate:
                # can be extended for RGBA values later
                points, keys = voxel_downsample(voxel_size, points, keys)
                keys = keys.reshape((keys.shape[0],))
                keys = keys[:, np.newaxis]

            pts_size_after = points.shape[0]

            points_down_removed += pts_size_before - pts_size_after
            points_saved += pts_size_after

            pts_keys_pair = np.append(points, keys, axis=1)
            points_keys_total = np.append(points_keys_total, pts_keys_pair, axis=0)

        def save_file(file_wo_ext: str, outfile_ext: str):
            nonlocal points_keys_total
            logger.info(f"Output file: {file_wo_ext + outfile_ext}")
            pc_status_print()

            if outfile_ext == ".ply" or outfile_ext == ".pcd":
                save_pointcloud(file_wo_ext + outfile_ext, points_keys_total, ascii, field)
            elif outfile_ext == ".las":
                LAS_file = laspy.create()
                LAS_file.x = points_keys_total[:, 0]
                LAS_file.y = points_keys_total[:, 1]
                LAS_file.z = points_keys_total[:, 2]
                # LAS file only has intensity but we can use it for other field
                # value
                LAS_file.intensity = points_keys_total[:, 3]
                LAS_file.write(file_wo_ext + outfile_ext)
                logger.info(f"LAS format only supports the Intensity field. "
                            f"Saving {field} as the Intensity field.")

            points_keys_total = np.empty(shape=[0, 4])

        def pc_status_print():
            nonlocal points_sum, points_down_removed, points_saved, points_out_range
            if points_sum == 0:
                logger.info("No points accumulated during this period.")
                return
            down_removed_pct = (points_down_removed / points_sum) * 100
            out_range_pct = (points_out_range / points_sum) * 100
            save_pct = (points_saved / points_sum) * 100
            logger.info(
                f"Point Cloud status info\n"
                f"{points_sum} points accumulated during this period,\n{points_down_removed} "
                f"down sampling points are removed [{down_removed_pct:.2f} %],\n{points_out_range} "
                f"out range points are removed [{out_range_pct:.2f} %],\n{points_saved} points "
                f"are saved [{save_pct:.2f} %].")
            points_sum = 0
            points_out_range = 0
            points_saved = 0
            points_down_removed = 0

        logger.info("Start processing...")
        nonlocal already_saved, empty_pose
        try:
            for scan_idx, scans in enumerate(scans_iter()):
                # if we saved after a loop, just quietly yield thereafter
                if already_saved:
                    yield scans
                    continue

                for idx, scan in enumerate(scans):
                    if scan is None:
                        continue

                    # Pose attribute is per col global pose so we use identity for scan
                    # pose
                    column_poses = scan.pose

                    if (empty_pose and column_poses.size > 0
                            and not np.array_equal(first_valid_column_pose(scan), np.eye(4))):
                        empty_pose = False

                    points = xyzlut_list[idx](scan)
                    keys = scan.field(field)

                    if scan_idx and scan_idx % 100 == 0:
                        logger.info(f"Processed {scan_idx} lidar scan")

                    # to remove out range points
                    valid_row_index = scan.field(ChanField.RANGE) > 0
                    out_range_row_index = scan.field(ChanField.RANGE) == 0
                    dewarped_points = dewarp(points, column_poses)
                    filtered_points = dewarped_points[valid_row_index]
                    filtered_keys = keys[valid_row_index]

                    curr_scan_points = dewarped_points.shape[0] * dewarped_points.shape[1]
                    points_sum += curr_scan_points
                    points_out_range += np.count_nonzero(out_range_row_index)

                    # may not need below line
                    shaped_keys = filtered_keys.reshape(filtered_keys.shape[0], 1)
                    points_to_process = np.append(points_to_process, filtered_points,
                                                  axis=0)
                    keys_to_process = np.append(keys_to_process, shaped_keys, axis=0)

                    # downsample the accumulated point clouds #
                    if scan_idx % down_sample_steps == 0:
                        process_points(points_to_process, keys_to_process)
                        points_to_process = np.empty(shape=[0, 3])
                        keys_to_process = np.empty(shape=[0, 1])
                        if verbose:
                            pc_status_print()

                # output a file to prevent crash due to oversize #
                if points_keys_total.shape[0] >= pts_per_file:
                    save_file(f"{file_wo_ext}-{file_numb:03}", outfile_ext)
                    file_numb += 1

                yield scans

        except KeyboardInterrupt:
            pass

        finally:
            if empty_pose:
                logger.info(
                    "Warning: Empty lidar scan pose in lidarscan stream!!!\n"
                    "Suggest: Append slam option to ouster-cli command or use a "
                    "SLAM output OSF file as an input.")

            # handle the last part of point cloud or the first part of point cloud if
            # the size is less than down_sample_steps
            if not already_saved:
                if keys_to_process.size > 0:
                    process_points(points_to_process, keys_to_process)

                if points_sum > 0:
                    save_file(f"{file_wo_ext}-{file_numb:03}", outfile_ext)
                logger.info("Finished point cloud saving.")
                already_saved = True

    ctx.scan_iter = convert_iter  # type: ignore


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

    def trajectory_dump(scan_source):
        with open(filename, "wt") as f:
            if tum:
                # TUM format
                f.write("#timestamp,x,y,z,qx,qy,qz,qw\n")
            else:
                # CSV format
                f.write("timestamp,x,y,z,qx,qy,qz,qw\n")

            for scans in scan_source():
                scan = scans[sensor_idx]
                if scan is None:
                    continue
                scan_ts = scan.get_first_valid_column_timestamp()
                scan_pose = first_valid_column_pose(scan)
                p = scan_pose[:3, 3]
                r = rotation_matrix_to_quaternion(scan_pose[:3, :3])

                if tum:
                    f.write(f"{scan_ts} {p[0]} {p[1]} {p[2]} "
                            f"{r[1]} {r[2]} {r[3]} {r[0]}\n")
                else:
                    f.write(f"{scan_ts},{p[0]},{p[1]},{p[2]},"
                            f"{r[1]},{r[2]},{r[3]},{r[0]}\n")
                yield scans

    # address generator type later
    print(f"Saving the trajectory to {filename} ...")
    ctx.scan_iter = partial(trajectory_dump, ctx.scan_iter)  # type: ignore


source.commands['ANY']['slam'] = source_slam
source.commands['ANY']['save_trajectory'] = save_trajectory
SourceSaveCommand.implementations[OusterIoType.PCD] = point_cloud_convert
SourceSaveCommand.implementations[OusterIoType.LAS] = point_cloud_convert
SourceSaveCommand.implementations[OusterIoType.PLY] = point_cloud_convert
