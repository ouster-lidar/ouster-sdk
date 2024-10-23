import os
import sys
import click
import logging
import laspy
import numpy as np
from ouster.sdk.io_type import OusterIoType
from ouster.cli.plugins.source import source  # type: ignore
from ouster.sdk.util import default_scan_fields
from ouster.cli.plugins.source_save import (SourceSaveCommand,
                                            determine_filename,
                                            create_directories_if_missing,
                                            _file_exists_error)
from ouster.sdk.client import (LidarScan,
                               ChanField,
                               XYZLut,
                               first_valid_column_ts,
                               first_valid_column_pose,
                               dewarp)
from ouster.cli.plugins.source_util import (source_multicommand,
                                            SourceCommandType,
                                            SourceCommandContext)


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('mapping')
_max_range = None
_min_range = None


@click.command
@click.option('--max-range', required=False, type=float,
              default=None, help="Discard points further than this distance in meters")
@click.option('--min-range', required=False, type=float,
              default=None, help="Discard points closer that this distance in meters")
@click.option('--percent-range', required=False, type=float,
              default=None, help="Discards points with a range greater than this "
              "percentile of ranges in the scan")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR)
def source_clip(ctx: SourceCommandContext, max_range: float,
                min_range: float, percent_range, **kwargs) -> None:
    """
    Clip points outside of a specified range. By default uses min/max ranges used
    by the preceding slam command if present.
    """

    min_range = _min_range if min_range is None else min_range
    max_range = _max_range if max_range is None else max_range

    if min_range is None and max_range is None and percent_range is None:
        raise click.exceptions.UsageError("You must provide --max-range, --min-range, or"
                                          "--range-percent to clip.")

    if percent_range is not None and (percent_range <= 0 or percent_range > 100):
        raise click.exceptions.UsageError(
                f"Expected 'percent_range' value to be between (0, 100], but received {percent_range}")

    low_range = min_range * 1000 if min_range else 0
    high_range = max_range * 1000 if max_range else float('inf')

    scans = ctx.scan_iter

    def clip_iter():
        for scanl in scans:
            out = []
            for scan in scanl:
                if scan is None:
                    out.append(None)
                    continue
                copy = LidarScan(scan)

                range1 = copy.field(ChanField.RANGE)

                if percent_range and percent_range != 100:
                    nonlocal high_range
                    non_zero_range1 = range1[range1 != 0]  # Filter out zeros
                    percentile_range = np.percentile(non_zero_range1, percent_range)
                    high_range = min(high_range, percentile_range)

                range1[(range1 < low_range) | (range1 > high_range)] = 0

                if copy.has_field(ChanField.RANGE2):
                    range2 = copy.field(ChanField.RANGE2)
                    range2[(range2 < low_range) | (range2 > high_range)] = 0
                out.append(copy)
            yield out

    ctx.scan_iter = clip_iter()


@click.command
@click.option('--max-range', required=False,
              default=150.0, help="Max valid range")
@click.option('--min-range', required=False,
              default=1.0, help="Min valid range")
@click.option('-v', '--voxel-size', required=False,
              type=float, help="Voxel map size")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR)
def source_slam(ctx: SourceCommandContext, max_range: float, min_range: float,
                voxel_size: float, **kwargs) -> None:
    """
    Run SLAM with a SOURCE.\n

    Example values for voxel_size:\n
        Outdoor: 0.8 - 1.5\n
        Large indoor: 0.4 - 0.8\n
        Small indoor: 0.1 - 0.5\n
    If voxel_size is not specifiied, the algorithm will use the first lidar scan to calculate it.\n
    Small voxel size could give more accurate results but take more memory and
    longer processing. For real-time slam, considing using a slightly larger voxel size
    and use visualizer to monitor the SLAM process.
    """

    global _max_range, _min_range
    _max_range = max_range
    _min_range = min_range

    def make_kiss_slam():
        try:
            from ouster.sdk.mapping.slam import KissBackend
        except ImportError as e:
            raise click.ClickException(click.style("kiss-icp, a package required for slam, is "
                                       f"unsupported on this platform. Error: {str(e)}", fg='red'))

        return KissBackend(infos=ctx.scan_source.metadata,
                           max_range=max_range,
                           min_range=min_range,
                           voxel_size=voxel_size,
                           live_stream=ctx.scan_source.is_live)

    try:
        slam_engine = make_kiss_slam()
    except (ValueError, click.ClickException) as e:
        logger.error(str(e))
        return

    def slam_iter(scan_source, slam_engine):
        scan_start_ts = None
        for scans in scan_source:
            # Use the first valid scan to check if the iteration restarts
            scan = scans[0]
            if scan is None:
                continue

            scan_ts = first_valid_column_ts(scan)
            if scan_ts == scan_start_ts:
                logger.info("SLAM restarts as scan iteration restarts")
                slam_engine = make_kiss_slam()
            if not scan_start_ts:
                scan_start_ts = scan_ts
            slam_scans = slam_engine.update(scans)
            yield slam_scans

    ctx.scan_iter = slam_iter(ctx.scan_iter, slam_engine)


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
@click.option('-f', '--pts-per-file', default=15000000, type=int,
              help="the number of points per output file. Default is 15000000")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def point_cloud_convert(ctx: SourceCommandContext, filename: str, prefix: str,
                        dir: str, voxel_size: float, field: str, decimate: bool,
                        overwrite: bool, verbose: bool, max_z: float,
                        min_z: float, pts_per_file: int, **kwargs) -> None:

    pcu_installed = False
    if decimate:
        try:
            import point_cloud_utils as pcu  # type: ignore
            pcu_installed = True
        except ImportError:
            logger.warning("The point_cloud_utils library is not supported or installed,"
                           "so the process will generate larger, non-downsampled files")

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

            if pcu_installed and decimate:
                # can be extended for RGBA values later
                points, keys = pcu.downsample_point_cloud_on_voxel_grid(
                    voxel_size, points, keys)
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

            if outfile_ext == ".ply":
                # CloudCompare PLY color point cloud using the range 0-1
                with open(file_wo_ext + outfile_ext, 'w') as f:
                    # Write PLY header
                    f.write("ply\n")
                    f.write("format ascii 1.0\n")
                    f.write(
                        "element vertex {}\n".format(
                            points_keys_total.shape[0]))
                    f.write("property float x\n")
                    f.write("property float y\n")
                    f.write("property float z\n")
                    f.write(f"property float {field}\n")
                    f.write("end_header\n")
                    for i in range(points_keys_total.shape[0]):
                        f.write("{} {} {} {}\n".format(
                            points_keys_total[i, 0], points_keys_total[i, 1],
                            points_keys_total[i, 2], points_keys_total[i, 3] / 255))
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
            elif outfile_ext == ".pcd":
                with open(file_wo_ext + outfile_ext, 'w') as f:
                    # Write PCD header
                    f.write(f"FIELDS x y z {field}\n")
                    f.write("SIZE 4 4 4 4\n")
                    f.write("TYPE F F F F\n")
                    f.write("COUNT 1 1 1 1\n")
                    f.write("WIDTH %d\n" % (points_keys_total.shape[0]))
                    f.write("HEIGHT 1\n")
                    f.write("POINTS %d\n" % (points_keys_total.shape[0]))
                    f.write("DATA ascii\n")
                    for i in range(points_keys_total.shape[0]):
                        f.write("{} {} {} {}\n".format(
                            points_keys_total[i, 0], points_keys_total[i, 1],
                            points_keys_total[i, 2], points_keys_total[i, 3]))

            points_keys_total = np.empty(shape=[0, 4])

        def pc_status_print():
            nonlocal points_sum, points_down_removed, points_saved, points_out_range
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

        scan_start_ts = None
        empty_pose = True
        finish_saving = False
        finish_saving_action = True
        logger.info("Start processing...")
        try:
            for scan_idx, scans in enumerate(scans_iter):
                for idx, scan in enumerate(scans):
                    if scan is None:
                        continue
                    scan_ts = first_valid_column_ts(scan)
                    if scan_ts == scan_start_ts:
                        finish_saving = True
                        logger.info("Scan iteration restarts")
                    if finish_saving:
                        # Save point cloud and printout when scan iteration restarts
                        # This action only do once
                        if finish_saving_action:
                            process_points(points_to_process, keys_to_process)
                            save_file(f"{file_wo_ext}-{file_numb:03}", outfile_ext)
                            logger.info("Finished point cloud saving.")
                            finish_saving_action = False
                        yield scan
                        continue
                    if not scan_start_ts:
                        scan_start_ts = scan_ts

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
            if not finish_saving:
                if keys_to_process.size > 0:
                    process_points(points_to_process, keys_to_process)

                save_file(f"{file_wo_ext}-{file_numb:03}", outfile_ext)
                logger.info("Finished point cloud saving.")

    ctx.scan_iter = convert_iter()


source.commands['ANY']['slam'] = source_slam
source.commands['ANY']['clip'] = source_clip
SourceSaveCommand.implementations[OusterIoType.PCD] = point_cloud_convert
SourceSaveCommand.implementations[OusterIoType.LAS] = point_cloud_convert
SourceSaveCommand.implementations[OusterIoType.PLY] = point_cloud_convert
