from typing import Optional, Tuple, Union, cast
from contextlib import closing
import os
import struct
import re
from io import StringIO
import click
import logging
import laspy
import numpy as np
from functools import partial
from ouster.sdk.core import OusterIoType, ScanSource, voxel_downsample
from ouster.cli.plugins.source import source  # type: ignore
from ouster.cli.plugins.source_save import (SourceSaveCommand,
                                            determine_filename,
                                            create_directories_if_missing,
                                            _file_exists_error)
from ouster.sdk.core import (ChanField,
                             LidarScan,
                             LidarScanSet,
                             XYZLut,
                             first_valid_column_pose,
                             dewarp)
from ouster.sdk import open_source
from ouster.cli.plugins.source_util import (source_multicommand,
                                            SourceCommandType,
                                            SourceCommandContext)
from ouster.sdk.util.extrinsics import rotation_matrix_to_quaternion     # type: ignore
from ouster.sdk.mapping import AbsolutePoseConstraint, SlamConfig, SlamEngine


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('mapping')


def parse_gps_constraints_weights(
        gps_constraints_weights: Optional[Union[str, Tuple[float, float, float]]] = None
) -> Tuple[float, float, float]:
    """Parse GPS constraint weights from CLI input.

    Accepts either a string in the form "WX,WY,WZ" (commas or spaces) or a
    pre-parsed 3-tuple. Returns a validated (WX, WY, WZ) tuple of floats.
    """
    if gps_constraints_weights is None:
        return (0.01, 0.01, 0.001)

    if isinstance(gps_constraints_weights, tuple):
        if len(gps_constraints_weights) != 3:
            raise ValueError(
                "Expected gps_constraints_weights to be a 3-tuple (WX, WY, WZ)")
        weights = tuple(float(w) for w in gps_constraints_weights)
    elif isinstance(gps_constraints_weights, str):
        parts = re.split(r"[,\s]+", gps_constraints_weights.strip())
        if len(parts) != 3:
            raise ValueError(
                "Expected 'WX,WY,WZ' (3 numbers), e.g. '0.01,0.01,0.001'")
        weights = tuple(float(p) for p in parts)
    else:
        raise TypeError(
            "gps_constraints_weights must be a 3-tuple or a 'WX,WY,WZ' string")

    if any(w < 0 for w in weights):
        raise ValueError("GPS constraint weights must be non-negative")

    return cast(Tuple[float, float, float], weights)


def save_pointcloud(filename: str, cloud: np.ndarray, ascii: bool = False, field="unknown"):

    def ply_header():
        header = []
        header.append("ply")
        header.append("format ascii 1.0" if ascii else "format binary_little_endian 1.0")
        header.append(f"element vertex {cloud.shape[0]}")
        header.append("property float x")
        header.append("property float y")
        header.append("property float z")
        # determine keys and normals
        keys_count = 3 if field.upper() == 'RGB' else 1
        normals_count = cloud.shape[1] - 3 - keys_count
        if normals_count not in (0, 3):
            raise ValueError("Unsupported cloud shape for PLY: expected normals to be 0 or 3 columns")

        if keys_count == 1:
            header.append(f"property float {field}")
        else:
            header.append("property uchar red")
            header.append("property uchar green")
            header.append("property uchar blue")

        if normals_count == 3:
            header.append("property float nx")
            header.append("property float ny")
            header.append("property float nz")

        header.append("end_header")
        return "\n".join(header) + "\n"

    def pcd_header(data_mode: str):
        header = []
        # determine fields dynamically
        keys_count = 3 if field.upper() == 'RGB' else 1
        normals_count = cloud.shape[1] - 3 - keys_count
        if normals_count not in (0, 3):
            raise ValueError("Unsupported cloud shape for PCD: expected normals to be 0 or 3 columns")

        fields = ["x", "y", "z"]
        sizes = [4, 4, 4]
        types = ["F", "F", "F"]
        counts = [1, 1, 1]

        if keys_count == 1:
            fields.append(field)
            sizes.append(4)
            types.append("F")
            counts.append(1)
        else:
            fields.append("rgb")
            sizes.append(4)
            types.append("F")
            counts.append(1)

        if normals_count == 3:
            fields.extend(["nx", "ny", "nz"])
            sizes.extend([4, 4, 4])
            types.extend(["F", "F", "F"])
            counts.extend([1, 1, 1])

        header.append(f"FIELDS {' '.join(fields)}")
        header.append(f"SIZE {' '.join(map(str, sizes))}")
        header.append(f"TYPE {' '.join(types)}")
        header.append(f"COUNT {' '.join(map(str, counts))}")
        header.append(f"WIDTH {cloud.shape[0]}")
        header.append("HEIGHT 1")
        header.append(f"POINTS {cloud.shape[0]}")
        header.append(f"DATA {data_mode}")
        return "\n".join(header) + "\n"

    def color_f2i(value):
        return int(np.clip(255 * value, 0, 255))

    if filename.endswith(".ply"):
        if ascii:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(ply_header())
                # infer keys and normals
                keys_count = 3 if field.upper() == 'RGB' else 1
                normals_count = cloud.shape[1] - 3 - keys_count
                for i in range(cloud.shape[0]):
                    x, y, z = cloud[i, 0], cloud[i, 1], cloud[i, 2]
                    if keys_count == 1:
                        key = cloud[i, 3]
                        if normals_count == 0:
                            f.write(f"{x} {y} {z} {key}\n")
                        else:
                            nx, ny, nz = cloud[i, 4], cloud[i, 5], cloud[i, 6]
                            f.write(f"{x} {y} {z} {key} {nx} {ny} {nz}\n")
                    else:
                        r = color_f2i(cloud[i, 3])
                        g = color_f2i(cloud[i, 4])
                        b = color_f2i(cloud[i, 5])
                        if normals_count == 0:
                            f.write(f"{x} {y} {z} {r} {g} {b}\n")
                        else:
                            nx, ny, nz = cloud[i, 6], cloud[i, 7], cloud[i, 8]
                            f.write(f"{x} {y} {z} {r} {g} {b} {nx} {ny} {nz}\n")
        else:
            with open(filename, 'wb') as f:  # type: ignore[assignment]
                f.write(ply_header().encode("ascii"))  # type: ignore[arg-type]
                # write full float32 buffer (includes normals if present)
                # if RGB keys present, convert to byte colors in binary case below
                keys_count = 3 if field.upper() == 'RGB' else 1
                normals_count = cloud.shape[1] - 3 - keys_count
                if keys_count == 1:
                    cloud_bin = cloud.copy().astype(np.float32)
                    f.write(cloud_bin.tobytes())  # type: ignore[arg-type]
                else:
                    # encode RGB as bytes per vertex
                    for i in range(cloud.shape[0]):
                        x, y, z = cloud[i, 0], cloud[i, 1], cloud[i, 2]
                        r = np.uint8(color_f2i(cloud[i, 3]))
                        g = np.uint8(color_f2i(cloud[i, 4]))
                        b = np.uint8(color_f2i(cloud[i, 5]))
                        if normals_count == 3:
                            nx, ny, nz = float(cloud[i, 6]), float(cloud[i, 7]), float(cloud[i, 8])
                            f.write(struct.pack('<3f3B3f', x, y, z, r, g, b, nx, ny, nz))  # type: ignore[arg-type]
                        else:
                            f.write(struct.pack('<3f3B', x, y, z, r, g, b))  # type: ignore[arg-type]

    elif filename.endswith(".pcd"):
        if ascii:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(pcd_header("ascii"))
                keys_count = 3 if field.upper() == 'RGB' else 1
                normals_count = cloud.shape[1] - 3 - keys_count

                def pack_rgb_to_float(r, g, b):
                    ri = color_f2i(r)
                    gi = color_f2i(g)
                    bi = color_f2i(b)
                    rgb_int = (ri << 16) | (gi << 8) | bi
                    return struct.unpack('<f', struct.pack('<I', rgb_int))[0]
                for i in range(cloud.shape[0]):
                    x, y, z = cloud[i, 0], cloud[i, 1], cloud[i, 2]
                    if keys_count == 1:
                        if normals_count == 0:
                            f.write(f"{x} {y} {z} {cloud[i,3]}\n")
                        else:
                            f.write(f"{x} {y} {z} {cloud[i,3]} {cloud[i,4]} {cloud[i,5]} {cloud[i,6]}\n")
                    else:
                        color = pack_rgb_to_float(cloud[i, 3], cloud[i, 4], cloud[i, 5])
                        if normals_count == 0:
                            f.write(f"{x} {y} {z} {np.float32(color)}\n")
                        else:
                            f.write(f"{x} {y} {z} {np.float32(color)} {cloud[i,6]} {cloud[i,7]} {cloud[i,8]}\n")
        else:
            with open(filename, 'wb') as f:  # type: ignore[assignment]
                buf = StringIO()
                buf.write(pcd_header("binary"))
                f.write(buf.getvalue().encode("ascii"))  # type: ignore[arg-type]

                # For binary, simply write the full float32 buffer. If RGB present, pack as before
                keys_count = 3 if field.upper() == 'RGB' else 1
                normals_count = cloud.shape[1] - 3 - keys_count
                if keys_count == 3:
                    # pack rgb into one float channel, preserve normals if present
                    r = np.clip((255 * cloud[:, 3]).astype(np.uint32), 0, 255)
                    g = np.clip((255 * cloud[:, 4]).astype(np.uint32), 0, 255)
                    b = np.clip((255 * cloud[:, 5]).astype(np.uint32), 0, 255)
                    rgb_u32 = (r << 16) | (g << 8) | b
                    rgb_f32 = rgb_u32.view(np.float32)
                    if normals_count == 0:
                        out = np.empty((cloud.shape[0], 4), dtype=np.float32)
                        out[:, 0:3] = cloud[:, 0:3].astype(np.float32)
                        out[:, 3] = rgb_f32
                        f.write(out.tobytes())  # type: ignore[arg-type]
                    else:
                        out = np.empty((cloud.shape[0], 7), dtype=np.float32)
                        out[:, 0:3] = cloud[:, 0:3].astype(np.float32)
                        out[:, 3] = rgb_f32
                        out[:, 4:7] = cloud[:, 6:9].astype(np.float32)
                        f.write(out.tobytes())  # type: ignore[arg-type]
                else:
                    f.write(cloud.astype(np.float32).tobytes())  # type: ignore[arg-type]


@click.command
@click.option('--max-range', required=False, show_default=True,
              default=150.0, help="Max valid range")
@click.option('--min-range', required=False, show_default=True,
              default=1.0, help="Min valid range")
@click.option('-v', '--voxel-size', required=False,
              type=float, help="Voxel map size (meters)")
@click.option('-d', '--dump-map', required=False,
              default="", type=str, help="Dumps the map to a ply file")
@click.option('--deskew-method', type=click.Choice(['auto', 'none', 'constant_velocity', 'imu_deskew']),
              default='auto', show_default=True,
              help="Method used for motion compensation (deskewing) of point clouds")
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR_UNREPEATABLE)
def source_slam(ctx: SourceCommandContext, max_range: float, min_range: float,
                voxel_size: float, dump_map: str, deskew_method: str) -> None:
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

    def make_kiss_slam() -> SlamEngine:

        config = SlamConfig()
        config.backend = "kiss"
        config.deskew_method = deskew_method
        config.min_range = min_range
        config.max_range = max_range
        config.voxel_size = voxel_size if voxel_size is not None else 0.0
        config.initial_pose = ctx.other_options.get("initial_pose", np.eye(4))

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
            points = slam_engine.get_point_cloud()
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
              "This parameter is the same as the open3D voxel size. "
              "The bigger the value, the fewer points it outputs", show_default=True)
@click.option('--field',
              required=False,
              type=click.Choice(['SIGNAL',
                                 'NEAR_IR',
                                 'REFLECTIVITY',
                                 'RGB'],
                                case_sensitive=False),
              default="REFLECTIVITY",
              help="Chanfield for output file key value.", show_default=True)
@click.option('--decimate', required=False, type=bool,
              default=True, help="Downsample the point cloud to output.", show_default=True)
@click.option('--verbose', is_flag=True, default=False,
              help="Print point cloud status much frequently.", show_default=True)
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite "
              "existing files with the same name")
@click.option('--max-z', default=None, type=float, help="Max z threshold for point cloud saving")
@click.option('--min-z', default=None, type=float, help="Min z threshold for point cloud saving")
@click.option('-f', '--pts-per-file', default=100000000, type=int, show_default=True,
              help="the number of points per output file.")
@click.option('--ascii', is_flag=True, default=False,
              help="Output files in ASCII rather than binary format")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def point_cloud_convert(ctx: SourceCommandContext, filename: str, prefix: str,
                        dir: str, voxel_size: float, field: str, decimate: bool,
                        overwrite: bool, verbose: bool, max_z: float,
                        min_z: float, pts_per_file: int, ascii: bool, **kwargs) -> None:

    scans_iter = ctx.scan_iter

    if ctx.scan_source is not None:
        infos = ctx.scan_source.sensor_info

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

    keys_count = 3 if field.upper() == "RGB" else 1
    normals_included = False

    def key_columns() -> int:
        return keys_count + (3 if normals_included else 0)

    def total_columns() -> int:
        return 3 + key_columns()

    def convert_iter():
        nonlocal already_saved, empty_pose
        points_to_process = np.zeros((0, 3), dtype=float)
        keys_to_process = np.zeros((0, key_columns()), dtype=float)
        points_keys_total = np.zeros((0, total_columns()), dtype=float)

        def enable_normals():
            nonlocal normals_included, keys_to_process, points_keys_total
            if normals_included:
                return
            normals_included = True
            if points_keys_total.size:
                points_keys_total = np.append(
                    points_keys_total,
                    np.zeros((points_keys_total.shape[0], 3), dtype=float),
                    axis=1)
            else:
                points_keys_total = np.zeros((0, total_columns()), dtype=float)
            if keys_to_process.size:
                keys_to_process = np.append(
                    keys_to_process,
                    np.zeros((keys_to_process.shape[0], 3), dtype=float),
                    axis=1)
            else:
                keys_to_process = np.zeros((0, key_columns()), dtype=float)

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
                points, keys = voxel_downsample(voxel_size, points, keys)

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

            points_keys_total = np.zeros((0, total_columns()), dtype=float)

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
                    normals_field = None
                    if scan.has_field(ChanField.NORMALS):
                        try:
                            candidate_normals = scan.field(ChanField.NORMALS)
                            if (candidate_normals is not None and
                                    candidate_normals.ndim >= 2 and
                                    candidate_normals.shape[-1] >= 3):
                                normals_field = candidate_normals
                                enable_normals()
                            else:
                                normals_field = None
                        except Exception:
                            normals_field = None

                    if scan_idx and scan_idx % 100 == 0:
                        logger.info(f"Processed {scan_idx} lidar scan")

                    # to remove out range points
                    valid_row_index = scan.field(ChanField.RANGE) > 0
                    out_range_row_index = scan.field(ChanField.RANGE) == 0
                    dewarped_points = dewarp(points, column_poses)
                    filtered_points = dewarped_points[valid_row_index]
                    filtered_keys = keys[valid_row_index]
                    if normals_included:
                        if normals_field is not None:
                            try:
                                filtered_normals = normals_field[valid_row_index]
                            except Exception:
                                filtered_normals = np.zeros((filtered_keys.shape[0], 3),
                                                            dtype=float)
                        else:
                            filtered_normals = np.zeros((filtered_keys.shape[0], 3),
                                                        dtype=float)

                    curr_scan_points = dewarped_points.shape[0] * dewarped_points.shape[1]
                    points_sum += curr_scan_points
                    points_out_range += np.count_nonzero(out_range_row_index)

                    points_to_process = np.append(points_to_process, filtered_points,
                                                  axis=0)
                    if filtered_keys.ndim == 1:
                        filtered_keys = filtered_keys.reshape(-1, 1)
                    if normals_included:
                        if filtered_normals.ndim == 1:
                            filtered_normals = filtered_normals.reshape(-1, 3)
                        combined = np.concatenate([filtered_keys, filtered_normals], axis=1)
                        keys_to_process = np.append(keys_to_process, combined, axis=0)
                    else:
                        keys_to_process = np.append(keys_to_process, filtered_keys, axis=0)

                    # downsample the accumulated point clouds #
                    if scan_idx % down_sample_steps == 0:
                        process_points(points_to_process, keys_to_process)
                        points_to_process = np.zeros((0, 3), dtype=float)
                        keys_to_process = np.zeros((0, key_columns()), dtype=float)
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


def relative_xy_from_wgs84(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    """Local XY in meters using a WGS84-based linearization around (lat0, lon0)."""
    earth_equator_radius = 6378137.0
    earth_eccentricity = 0.08181919084261

    lat0_rad = np.deg2rad(lat0)
    lon0_rad = np.deg2rad(lon0)
    lat_rad = np.deg2rad(lat)
    lon_rad = np.deg2rad(lon)

    p = earth_eccentricity * np.sin(lat0_rad)
    p = 1.0 - p * p
    rho_e = earth_equator_radius * (1.0 - earth_eccentricity * earth_eccentricity) / (np.sqrt(p) * p)
    rho_n = earth_equator_radius / np.sqrt(p)
    rho_lat = rho_e
    rho_lon = rho_n * np.cos(lat0_rad)

    d_lat = (lat_rad - lat0_rad) * rho_lat
    d_lon = (lon_rad - lon0_rad) * rho_lon

    x = d_lon
    y = d_lat
    return x, y


def add_auto_gps_constraints(po,
                             source_name: str,
                             gps_constraints_every_m: float,
                             gps_constraints_weights: Optional[Union[str, Tuple[float, float, float]]] = None) -> int:
    """Generate GPS AbsolutePoseConstraint(s) and add them to the given PoseOptimizer.

    Args:
        po: PoseOptimizer instance to add constraints to.
        source_name: OSF path.
        gps_constraints_every_m: Add constraints roughly every N meters traveled,
            computed from LidarScan poses.
        gps_constraints_weights: Per-axis translation weights WX, WY, WZ. May
            be provided as a tuple or as a string "WX,WY,WZ" (commas or spaces).
            If the scan pose is identity, WZ is forced to 0 because no altitude
            can be inferred from the pose track.
    """
    gps_constraints_weights = parse_gps_constraints_weights(gps_constraints_weights)
    logger.info("Applying auto GPS constraint translation weights (WX,WY,WZ)=%s",
                gps_constraints_weights)

    if gps_constraints_every_m <= 0:
        raise ValueError("gps_constraints_every_m must be > 0")

    with closing(open_source(source_name, index=True, sensor_idx=0)) as source:
        num_scans = len(source)
        if num_scans <= 0:
            raise RuntimeError("No scans found in the source")

        if num_scans <= 1:
            raise RuntimeError("Not enough scans to generate GPS constraints (need at least 2)")

        def get_scan(frame) -> Optional[LidarScan]:
            if isinstance(frame, LidarScan):
                return frame
            if isinstance(frame, LidarScanSet):
                if len(frame) > 0:
                    scan0 = frame[0]
                    return scan0 if isinstance(scan0, LidarScan) else None
                return None
            if isinstance(frame, (list, tuple)) and frame:
                scan0 = frame[0]
                return scan0 if isinstance(scan0, LidarScan) else None
            return None

        def get_gps_lat_lon(scan: LidarScan) -> Tuple[float, float]:
            lat_lon = scan.field("POSITION_LAT_LONG")
            lat, lon = lat_lon[-1, :]
            return float(lat), float(lon)

        def get_gps_ts(scan: LidarScan) -> np.uint64:
            ts = scan.field("POSITION_TIMESTAMP")[-1]
            return np.uint64(ts)

        added = 0
        lat0 = None
        lon0 = None
        has_gps_fields = False

        prev_pos_xy: Optional[np.ndarray] = None
        distance_since_last_constraint_m = float("inf")

        for idx in range(1, num_scans):
            scan = get_scan(source[int(idx)])
            if scan is None:
                continue

            if scan.has_field("POSITION_LAT_LONG") and scan.has_field("POSITION_TIMESTAMP"):
                has_gps_fields = True

            try:
                scan_pose = first_valid_column_pose(scan)
            except Exception:
                scan_pose = np.eye(4)

            is_identity = np.allclose(scan_pose, np.eye(4), atol=1e-6)
            if not is_identity:
                pos_xy = np.array(
                    [float(scan_pose[0, 3]), float(scan_pose[1, 3])],
                    dtype=float,
                )
                if prev_pos_xy is not None:
                    distance_since_last_constraint_m += float(np.linalg.norm(pos_xy - prev_pos_xy))
                prev_pos_xy = pos_xy

            if added > 0 and distance_since_last_constraint_m < gps_constraints_every_m:
                continue

            try:
                lat, lon = get_gps_lat_lon(scan)
                ts = get_gps_ts(scan)
            except Exception:
                continue

            pose = np.eye(4)
            # GPS provides lat/lon only; we constrain XY. If the scan has a
            # non-identity pose track we also copy its Z translation and apply a
            # (typically small) Z weight; otherwise we set WZ to 0.
            if lat0 is None or lon0 is None:
                lat0, lon0 = lat, lon
            pose[:2, -1] = relative_xy_from_wgs84(lat, lon, lat0, lon0)
            if not is_identity:
                pose[2, 3] = float(scan_pose[2, 3])

            wz = 0.0 if is_identity else float(gps_constraints_weights[2])
            translation_weights = np.array(
                [gps_constraints_weights[0], gps_constraints_weights[1], wz],
                dtype=float,
            )

            constraint = AbsolutePoseConstraint(
                timestamp=ts,
                pose=pose,
                rotation_weight=0.0,
                translation_weight=translation_weights)
            po.add_constraint(constraint)
            added += 1
            distance_since_last_constraint_m = 0.0

        if added == 0 and not has_gps_fields:
            logger.warning(
                "No GPS fields POSITION_LAT_LONG/POSITION_TIMESTAMP found in source %s; "
                "skipping auto GPS constraints.",
                source_name,
            )

        return added


source.commands['ANY']['slam'] = source_slam
source.commands['ANY']['save_trajectory'] = save_trajectory
SourceSaveCommand.implementations[OusterIoType.PCD] = point_cloud_convert
SourceSaveCommand.implementations[OusterIoType.LAS] = point_cloud_convert
SourceSaveCommand.implementations[OusterIoType.PLY] = point_cloud_convert
