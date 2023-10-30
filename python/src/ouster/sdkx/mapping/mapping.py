# type: ignore
import sys
import click
import ipaddress
from typing import Optional
from ouster.cli.core import cli
from ouster.cli.core.util import click_ro_file
import ouster.sdk.pose_util as pu
from ouster.sdk.util import resolve_metadata
from ouster import client
from pathlib import Path
import ouster.osf as osf
import ouster.sdkx.mapping.util as util
from ouster.sdkx.parsing import default_scan_fields
from datetime import datetime
import time


@cli.group(name="mapping")
def mapping_group() -> None:
    """Mapping tools."""
    pass


class SLAMOSFWriter:

    def __init__(self, source: str, output_path: str, chunk_size: int, lidar_port: int, imu_port: int, meta: str):
        source_ext = Path(source).suffix
        if source_ext == ".pcap":
            if meta:
                meta_data = meta
            else:
                meta_data = resolve_metadata(source)
            if not meta_data:
                raise Exception("File not found, please specify a metadata file with `-m`")
            with open(Path(meta_data)) as meta_file:
                meta_json = meta_file.read()
                lidar_sensor_meta = osf.LidarSensor(meta_json)
        elif source_ext == ".local" or ipaddress.ip_address(source):
            print(f"Connecting to {source}")
            sensor_source = client.Sensor(source, lidar_port, imu_port)
            lidar_sensor_meta = osf.LidarSensor(sensor_source._fetched_meta)
        else:
            raise ValueError("Off")

        self.writer = osf.Writer(output_path, "SLAM_Output_OSF", chunk_size)
        # TODO sensor_id may changed depends on how will we handle multi live sensor
        sensor_osf_id = dict()
        # Fix the sensor id logic. Later need to extend to multisensor
        single_sensor_id = 0
        sensor_osf_id[single_sensor_id] = self.writer.addMetadata(lidar_sensor_meta)

        self.lidar_stream = osf.LidarScanStream(self.writer, sensor_osf_id[single_sensor_id])

    def write_scan(self, scan_ts, scan):
        self.lidar_stream.save(scan_ts, scan)

    def close(self):
        self.writer.close()


def slam_scans_generator(scan_source, slam, osf_writer):
    for scan in scan_source:
        scan_slam = slam.update(scan)
        scan_ts = client.first_valid_packet_ts(scan_slam)
        osf_writer.write_scan(scan_ts, scan_slam)
        yield scan_slam

    osf_writer.close()


@mapping_group.command(name='slam')
@click.argument('source', required=True, type=str)
@click.argument('viz', required=False, default="", type=str)
@click.option('-m', '--meta', required=False, default=None, type=click_ro_file,
              help="Metadata file for pcap input")
@click.option('--slam_name', default='kiss_slam', help="Slam name")
@click.option('-l', '--lidar_port', default=7502, help="Lidar port")
@click.option('-i', '--imu_port', default=7503, help="IMU port")
@click.option('-o', '--output', required=False, help="OSF output filename")
@click.option("--accum-num",
              default=0,
              help="Integer number of scans to accumulate")
@click.option("--accum-every",
              default=None,
              type=float,
              help="Accumulate every Nth scan")
@click.option("--accum-every-m",
              default=None,
              type=float,
              help="Accumulate scan every M meters traveled")
@click.option("--accum-map",
              is_flag=True,
              help="Enable the overall map accumulation mode")
@click.option("--accum-map-ratio",
              default=0.001,
              help="Ratio of random points of every scan to add to an overall map")
def run_slam(
    source: str,
    viz: str,
    meta: Optional[str],
    slam_name: str,
    lidar_port: int,
    imu_port: int,
    output: Optional[str],
    accum_num: int,
    accum_every: Optional[int],
    accum_every_m: Optional[float],
    accum_map: bool,
    accum_map_ratio: float) -> None:
    """
    Run SLAM algorithm with an optional visualizer

    Run with a sensor or a pcap file to produce an OSF containing the lidar data and SLAM poses.
    To turn on visualizer, append 'viz' or 'visualizer' to the command, case insensitive.
    """
    run_slam_impl(source, viz, meta, slam_name, lidar_port, imu_port, output,
                  accum_num, accum_every, accum_every_m, accum_map,
                  accum_map_ratio)


def run_slam_impl(source: str,
                  viz: str = None,
                  meta: str = None,
                  slam_name: str = "kiss_slam",
                  lidar_port: int = 7502,
                  imu_port: int = 7503,
                  output: str = None,
                  accum_num: int = 0,
                  accum_every: Optional[int] = None,
                  accum_every_m: Optional[float] = None,
                  accum_map: bool = False,
                  accum_map_ratio: float = 0.001) -> None:

    data_source = util.Source(source, meta = meta)

    if not output:
        date_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        metadata = data_source.metadata
        output = f"{metadata.prod_line}_{metadata.sn}_{metadata.fw_rev}_{metadata.mode}_{date_time}_slam_output.osf"

    if slam_name == "kiss_slam":
        try:
            from ouster.sdkx.mapping.slam import KissBackend
        except ImportError as e:
            raise click.ClickException("kiss_icp is not installed. Please run pip install kiss-icp. Error: " + str(e))
        slam = KissBackend(info=data_source.metadata)
    else:
        raise ValueError("Only support KISS-ICP SLAM for now")

    chunk_size = 0
    osf_writer = SLAMOSFWriter(source, output, chunk_size, lidar_port, imu_port, meta)

    print(f"Running {slam_name} SLAM and start writing LidarScan and Traj into {output}\nhit ctrl-c to exit")

    start_time = time.time()
    slam_scan_gen = slam_scans_generator(data_source, slam, osf_writer)

    if viz and viz.lower() in ['viz', 'visualizer']:
        try:
            from ouster.viz import SimpleViz, scans_accum_for_cli
        except ImportError as e:
            raise click.ClickException("Error: " + str(e))
        scans_accum = scans_accum_for_cli(data_source.metadata,
                                          accum_num=accum_num,
                                          accum_every=accum_every,
                                          accum_every_m=accum_every_m,
                                          accum_map=accum_map,
                                          accum_map_ratio=accum_map_ratio)
        simple_viz = SimpleViz(data_source.metadata, scans_accum=scans_accum)
        simple_viz.run(slam_scan_gen)
    else:
        for _ in slam_scan_gen:
            pass

    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Elapsed time: {elapsed_time} seconds")


@mapping_group.command(name='convert')
@click.argument('input_file', required=True, type=click_ro_file)
@click.argument('output_file', required=True)
@click.option('-d', '--min_dist', default=2.0, help="Min dist (m) for points to "
        "save. Default value is 2m")
@click.option('-s', '--voxel_size', default=0.1, help="Voxel map size for downsampling."
        "This parameter is the same as the open3D voxel size. Default value is 0.1. "
        "The bigger the value, the fewer points it outputs"
        " http://www.open3d.org/docs/0.6.0/python_api/open3d.geometry.voxel_down_sample.html")
@click.option('-f',
    '--field',
    required=False,
    type=click.Choice(['SIGNAL',
    'NEAR_IR',
    'REFLECTIVITY'],
    case_sensitive=False),
    default="REFLECTIVITY",
    help="Chanfield for output file key value. Choose between SIGNAL, NEAR_IR, "
    "REFLECTIVITY. Default field is REFLECTIVITY")
@click.option('--print_process', required=False, type=bool, default=True, help="Default is On")
@click.option('--verbose_print', required=False, type=bool, default=False,
              help="Print point cloud status much frequently. Default is Off")
def point_cloud_convert(input_file: str, output_file: str, min_dist: float,
                       voxel_size: float, field: str,
                       print_process: Optional[bool], verbose_print: Optional[bool]) -> None:
    """
    Save point cloud from an OSF file into specific formats

    Output file format depends on output filename extension. The valid output files
    extensions are .las, .ply and .pcd. Default output format is .ply. For large point
    cloud, the output will be split into multiple files and each file is around 1G.
    Currently this tool only supports single lidar OSF files.
    """
    try:
        import open3d as o3d
        import numpy as np
        import laspy
    except ImportError:
        raise click.ClickException("Please verify that open3d, laspy libs are installed")

    # If not specify output filename and format, use input filename and ply format

    output_file_path = Path(output_file)
    file_wo_ext = str(output_file_path.stem)
    outfile_ext = str(output_file_path.suffix)
    if outfile_ext not in [".las", ".ply", ".pcd"]:
        sys.exit("Error: output file extension only support .las, .ply, and "
                 f".pcd, but input file extension is {outfile_ext}")

    points_for_downsample = np.empty(shape = [0, 3])
    points_ds_keys = np.empty(shape = [0, 1])

    pcd = o3d.t.geometry.PointCloud()
    points_total = o3d.t.geometry.PointCloud()

    # hard-coded parameters and counters #
    # affect the running time. smaller value mean longer running time. Too large
    # may lead to crash
    down_sample_steps = 100
    # affect per output file size. 30000000 makes output file size ~1G
    max_pnt_per_file = 30000000
    file_numb = 1
    # if enabled, print process every 5%
    parts_cnt = 20
    # will be updated using the total msgs
    delta_cnt = 0

    # variables for point cloud status printout
    points_sum = 0
    points_saved = 0
    points_zero = 0
    points_near_removed = 0
    points_down_removed = 0

    infile_ext = str(Path(input_file).suffix)
    if infile_ext not in [".osf"]:
        sys.exit("Exit! input file extension only support .osf files "
                 f"but input file extension is {infile_ext}")

    reader = osf.Reader(input_file)

    scans = osf.Scans(input_file)
    sensor_meta = reader.meta_store.get(osf.LidarSensor)
    info = sensor_meta.info
    sensor_id = sensor_meta.id

    valid_fields = default_scan_fields(info.format.udp_profile_lidar, flags=False)
    channel_field = client.ChanField.from_string(field)

    if channel_field not in valid_fields:
        valid_fields_str = ", ".join(map(str, list(valid_fields.keys())))

        sys.exit(f"Exit! field {field} is not available in the low bandwidth mode\n"
                 f"use -f and choose a valid field from {valid_fields_str}")

    scan_streams = reader.meta_store.find(osf.LidarScanStream)
    sensor_stream_id = next((mid for mid, m in scan_streams.items()
                             if m.sensor_meta_id == sensor_id), 0)

    streaming_info = reader.meta_store.get(osf.StreamingInfo)
    if streaming_info:
        for stream_id, stream_stat in streaming_info.stream_stats:
            if stream_id == sensor_stream_id:
                total_msgs = stream_stat.message_count
                break

    if total_msgs == 0:
        sys.exit("Exit! No lidar scan msg found.")

    delta_cnt = total_msgs // parts_cnt
    xyzlut = client.XYZLut(info, use_extrinsics=True)

    def down_sample_points(points_for_downsample, points_ds_keys):
        nonlocal points_total, pcd, points_down_removed, points_saved
        pcd.point.positions = o3d.core.Tensor(points_for_downsample)
        # normalized the key
        points_ds_keys *= 1 / points_ds_keys.max()
        pcd.point[field] = o3d.core.Tensor(points_ds_keys)
        down_points = pcd.voxel_down_sample(voxel_size)
        points_down_removed += (len(points_for_downsample) - down_points.point.positions.shape[0])
        points_saved += down_points.point.positions.shape[0]
        if points_total.is_empty():
            points_total = down_points
        else:
            points_total = points_total.append(down_points)
        pcd.clear()

    def save_file(file_wo_ext: str, outfile_ext: str, points_total):
        print(f"Output file: {file_wo_ext + outfile_ext}")

        pc_status_print()
        if outfile_ext == ".ply":
            # PLY intensity range is 0-1
            o3d.t.io.write_point_cloud((file_wo_ext + outfile_ext), points_total)
        elif outfile_ext == ".las":
            pos_array = points_total.point.positions[:, :].numpy()
            # LAS intensity range is 0-255
            key_array = points_total.point[field][:].numpy() * 255
            LAS_file = laspy.create()
            LAS_file.x = pos_array[:, 0]
            LAS_file.y = pos_array[:, 1]
            LAS_file.z = pos_array[:, 2]
            # LAS file only has intensity but we can use it for other field value
            LAS_file.intensity = key_array[:, 0]
            LAS_file.write(file_wo_ext + outfile_ext)
        elif outfile_ext == ".pcd":
            pos_array = points_total.point.positions[:, :].numpy()
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pos_array)
            o3d.io.write_point_cloud((file_wo_ext + outfile_ext), pcd)

    def pc_status_print():
        nonlocal points_sum, points_near_removed, points_down_removed, points_saved, points_zero
        near_minus_zero = points_near_removed - points_zero
        near_removed_pernt = (near_minus_zero / points_sum) * 100
        down_removed_pernt = (points_down_removed / points_sum) * 100
        zero_pernt = (points_zero / points_sum) * 100
        save_pernt = (points_saved / points_sum) * 100
        print(
            f"{points_sum} points accumulated during this period,\n{near_minus_zero} "
            f"near points are removed [{near_removed_pernt:.2f} %],\n{points_down_removed} "
            f"down sampling points are removed [{down_removed_pernt:.2f} %],\n{points_zero} "
            f"zero range points are removed [{zero_pernt:.2f} %],\n{points_saved} points "
            f"are saved [{save_pernt:.2f} %].")
        points_sum = 0
        points_zero = 0
        points_saved = 0
        points_near_removed = 0
        points_down_removed = 0

    empty_pose = True
    for scan_idx, scan in enumerate(scans):
        # Pose attribute is per col global pose so we use identity for scan pose
        column_poses = scan.pose

        if (empty_pose and column_poses.size > 0
                and not np.array_equal(column_poses[client.first_valid_column(scan)], np.eye(4))):
            empty_pose = False

        points = xyzlut(scan)
        keys = scan.field(channel_field)
        if delta_cnt != 0 and (scan_idx + 1) % delta_cnt == 0 and (100 // parts_cnt) * \
            ((scan_idx + 1) // delta_cnt) <= 100:
            print(f"{(100//parts_cnt) * ((scan_idx+1)//delta_cnt)} % of data processed")

        # to remove near points
        row_index = scan.field(client.ChanField.RANGE) > (min_dist * 1000)
        zero_row_index = scan.field(client.ChanField.RANGE) == 0
        dewarped_points = pu.dewarp(points, column_poses=column_poses)
        filtered_points = dewarped_points[row_index]
        filtered_keys = keys[row_index]

        curr_scan_points = row_index.shape[0] * row_index.shape[1]
        points_sum += curr_scan_points
        points_near_removed += curr_scan_points - np.count_nonzero(row_index)
        points_zero += np.count_nonzero(zero_row_index)

        shaped_keys = filtered_keys.reshape(filtered_keys.shape[0], 1)
        points_for_downsample = np.append(points_for_downsample, filtered_points,
                                          axis = 0)
        points_ds_keys = np.append(points_ds_keys, shaped_keys, axis = 0)

        # downsample the accumulated point clouds #
        if scan_idx % down_sample_steps == 0:
            down_sample_points(points_for_downsample, points_ds_keys)
            points_for_downsample = np.empty(shape = [0, 3])
            points_ds_keys = np.empty(shape = [0, 1])
            if verbose_print:
                pc_status_print()

        # output a file to prevent crash due to oversize #
        if points_total.is_empty() is False and points_total.point.positions.shape[0] >= max_pnt_per_file:
            save_file(file_wo_ext + str(file_numb), outfile_ext, points_total)
            file_numb += 1
            points_total.clear()

    # handle the last part of point cloud or the first part of point cloud if
    # the size is less than down_sample_steps
    if points_ds_keys.size > 0:
        down_sample_points(points_for_downsample, points_ds_keys)

    if empty_pose:
        print(
            "Warning: Empty lidar scan pose in the OSF file.\n"
            "Suggest: Use SLAM output OSF file for conversion. By command: ouster-cli mapping slam FILE")

    save_file(file_wo_ext + str(file_numb), outfile_ext, points_total)
