import logging
import os
import shutil
import sys
import subprocess
import shlex
from collections import deque
from pathlib import Path
from typing import Deque, Union, cast

from pcaps_to_osf import pcaps_to_osfs  # type: ignore

import ouster.sdk.util.pose_util as pu
from ouster.sdk.open_source import open_source
from ouster.sdk import client, osf

try:
    import numpy as np
    import open3d
except ImportError:
    raise

logger = logging.getLogger()
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('mapping_tool')

MIN_DIST = 3  # meter
MIN_Z = 2  # meter
MAX_Z = 12  # meter
PC_MATCH_CHECK = True
VOXEL_SIZE = 1.8

PathSource = Union[str, Path]
Output = Union[str, Path]
Meta = Union[str, Path]


def compute_unit_normals(dewarped_points, scan, info):
    """
    Use 2D depth image to calculate unit normal value for each valid pixel
    Ignore range less than mis_dist

    return a stagger normals value image
    """
    normals = np.zeros_like(dewarped_points)
    ranges = client.destagger(info, scan.field(client.ChanField.RANGE))
    destagger_points = client.destagger(info, dewarped_points)

    for u in range(scan.h):
        for v in range(scan.w):
            if ranges[u][v] < MIN_DIST:
                continue
            above = (max(0, u - 1), v)
            below = (min(scan.h - 1, u + 1), v)
            left = (u, (v - 1 + scan.w) % scan.w)
            right = (u, (v + 1) % scan.w)

            if (ranges[above[0]][above[1]] < MIN_DIST or
                ranges[below[0]][below[1]] < MIN_DIST or
                ranges[left[0]][left[1]] < MIN_DIST or
                    ranges[right[0]][right[1]] < MIN_DIST):
                continue
            diff_u = destagger_points[above[0]][above[1]
                                                ] - destagger_points[below[0]][below[1]]
            diff_v = destagger_points[left[0]][left[1]
                                               ] - destagger_points[right[0]][right[1]]
            normal = np.cross(diff_u, diff_v)
            n = np.sqrt(np.sum(normal ** 2))
            if n == 0:
                continue
            unit_normal = normal / n
            normals[u][v] = unit_normal

    stagger_normals = client.destagger(info, normals, True)
    return stagger_normals


def scans_to_cloud(scans: Deque[client.LidarScan],
                   info, xyzlut, target_pose=np.eye(4)):
    """
    Convert LidarScan to open3D PointCloud
    Filter out point which range is less than min_range and add normal value for valid points

    """
    cloud = open3d.geometry.PointCloud()
    comb_points = np.empty((0, 3))
    comb_normals = np.empty((0, 3))

    for scan in scans:
        local_points = xyzlut(scan)

        # filter out ground and ceiling for better alignment
        flat_local_points = local_points.reshape(
            local_points.shape[0] * local_points.shape[1], 3)
        filter_idx = (flat_local_points[:, 2] >= MIN_Z) & (
            flat_local_points[:, 2] <= MAX_Z)
        global_points = pu.dewarp(
            local_points,
            scan_pose=target_pose,
            column_poses=scan.pose)
        flat_global_points = global_points.reshape(
            global_points.shape[0] * global_points.shape[1], 3)
        filtered_points = flat_global_points[filter_idx]

        # normal calculation
        normals = compute_unit_normals(global_points, scan, info)
        flat_normals = normals.reshape(normals.shape[0] * normals.shape[1], 3)
        filtered_normals = flat_normals[filter_idx]

        comb_points = np.vstack((comb_points, filtered_points))
        comb_normals = np.vstack((comb_normals, filtered_normals))

    cloud.points = open3d.utility.Vector3dVector(comb_points)
    cloud.normals = open3d.utility.Vector3dVector(comb_normals)
    return cloud


def apply_icp(scan, icp_diff, last_pose):
    """
    Update scan per column pose using icp results

    """
    pose_corrected = np.matmul(icp_diff, np.matmul(last_pose, scan.pose))
    scan.pose[:] = pose_corrected


def register_two_scans(path, target_scan, source_scan, xyzlut,
                       target_pose, info, idx, debug=True) -> np.ndarray:
    """
    Update scan per column pose using icp results

    """
    target_cloud = scans_to_cloud(target_scan, info, xyzlut)
    source_cloud = scans_to_cloud(source_scan, info, xyzlut, target_pose)

    max_correspondence_distance = 4
    criteria = open3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-12,
                                                                    relative_rmse=1e-12,
                                                                    max_iteration=900)
    icp_result = open3d.pipelines.registration.registration_icp(
        source_cloud, target_cloud, max_correspondence_distance,
        estimation_method=open3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=criteria)

    result_cloud = source_cloud.transform(icp_result.transformation)

    """
    TODO remove the printout after development
    """
    if debug:
        logger.debug(f"transformation = {icp_result.transformation}")
        open3d.io.write_point_cloud(str(path) +
                                    "/target_cloud_normal" +
                                    str(idx) +
                                    ".ply",
                                    target_cloud)
        open3d.io.write_point_cloud(str(path) +
                                    "/source_cloud_normal" +
                                    str(idx) +
                                    ".ply",
                                    source_cloud)
        open3d.io.write_point_cloud(str(path) +
                                    "/result_cloud_normal" +
                                    str(idx) +
                                    ".ply",
                                    result_cloud)

    return icp_result.transformation

# Have this function to avoid A010.ply < A02.ply case


def sort_key(file_name):
    name, ext = os.path.splitext(file_name)
    numeric_part = ''.join(filter(str.isdigit, name))
    return (name.replace(numeric_part, ''), int(
        numeric_part)) if numeric_part else (file_name, 0)


def stitch_osfs(osf_path: PathSource, output: Output):
    """
    Stitch OSF files into one.

    Use the last N scans and the first N scans to stitch clouds.
    This function works only for single lidar setups.

    Args:
        osf_path (Path, str): Path to the directory containing OSF files.
        output (Path, str): Path to the output file.
    """
    osf_path = Path(osf_path)
    output = Path(output)
    if osf_path.is_dir():
        osf_files = list(osf_path.glob("*.osf"))
    else:
        logger.error("Expect a directory which contains OSF files")
        return

    logger.info(f"Combining {len(osf_files)} OSF files")

    # Sort the list of OSF files
    sorted_osf_files = sorted(osf_files, key=sort_key)

    fields_to_write = [client.ChanField.RANGE, client.ChanField.SIGNAL, client.ChanField.REFLECTIVITY]

    # peak the first OSF file
    peak_reader = open_source(str(sorted_osf_files[0]))
    info = peak_reader.metadata
    del peak_reader
    xyzlut = client.XYZLut(info, use_extrinsics=True)  # type: ignore
    osf_writer = osf.Writer(str(output), info, fields_to_write)

    # large max_scans could improve the stitch accuracy but increase the
    # processing time
    max_scans = 5
    last_pose = np.eye(4)
    icp_diff = np.eye(4)
    first_scan = True
    last_scans_deque: Deque[client.LidarScan] = deque(maxlen=max_scans)
    curr_scans_deque: Deque[client.LidarScan] = deque(maxlen=max_scans)

    for file_id, file in enumerate(sorted_osf_files):
        logger.info(f"Processing #{file_id} OSF file {file}")
        reader = osf.Reader(str(file))
        for msg in reader.messages():
            if msg.of(osf.LidarScanStream):
                # Skip the first scan from second OSF file as the first scan per
                # column poses are all eye(4) when SLAM restart. It will affect
                # ICP matches
                if file_id != 0 and first_scan:
                    first_scan = False
                    continue

                obj = msg.decode()
                scan = cast(client.LidarScan, obj)
                if file_id != 0:
                    if len(curr_scans_deque) < max_scans:
                        curr_scans_deque.append(scan)
                        continue

                    if np.all(icp_diff == np.eye(4)):
                        last_scan = last_scans_deque[-1]
                        last_pose = last_scan.pose[client.first_valid_column(
                            last_scan)]
                        icp_diff = register_two_scans(osf_path, last_scans_deque, curr_scans_deque,
                                                      xyzlut, last_pose, info, file_id, PC_MATCH_CHECK)
                        for stored_scan in curr_scans_deque:
                            apply_icp(stored_scan, icp_diff, last_pose)
                            osf_writer.save(0, stored_scan)
                            last_scans_deque.append(stored_scan)
                    apply_icp(scan, icp_diff, last_pose)

                osf_writer.save(0, scan)
                last_scans_deque.append(scan)

        icp_diff = np.eye(4)
        curr_scans_deque.clear()
        first_scan = True

    osf_writer.close()


def process_osf_files(osf_path: PathSource, output: Output):
    """
    Run slam on given OSF files.
    This function works only for single lidar setups.

    Args:
        osf_path (Path, str): Path of the directory containing osf files.
        output (Path, str): Path to the output file.
    """
    osf_path = Path(osf_path)
    output = Path(output)
    if osf_path.is_dir():
        osf_files = list(osf_path.glob("*.osf"))
    elif osf_path.is_file():
        osf_files = [osf_path]
    else:
        logger.error(
            f"Input {osf_path} is neither a directory nor a file")
        return

    sorted_osf_files = sorted(osf_files, key=sort_key)
    logger.info(f"Start to process {len(osf_files)} osf files")

    file_wo_ext = str(output.stem)
    for idx, _osf in enumerate(sorted_osf_files):
        osf_output_file = os.path.join(
            output.parent, file_wo_ext + str(idx) + '.osf')
        execution_command = f"ouster-cli source {_osf} slam -v {VOXEL_SIZE} save --ts lidar {osf_output_file}"
        process_slam = subprocess.run(
                shlex.split(execution_command),
                text=True,
                capture_output=True)
        logger.info(process_slam.stdout)
        if process_slam.returncode != 0:
            logger.error(f"Error executing slam command: {process_slam.stderr}")
            sys.exit(1)
        else:
            logger.info("Save SLAM output {osf_output_file}")


def delete_and_create_folder(folder_path):
    path_obj = Path(folder_path)

    if path_obj.exists():
        shutil.rmtree(folder_path)
        logger.info(f"Old result '{folder_path}' deleted")

    path_obj.mkdir(parents=True, exist_ok=True)
    logger.info(f"New folder {folder_path} created for result saving")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        logger.error(
            "Usage: python3 stitich.py PATH/data.pcap PATH/output.osf \nExit")
        exit(1)

    logger.info(f"pcap file {sys.argv[1]}")
    logger.info(f"output file {sys.argv[2]}")

    input_path = Path(sys.argv[1])
    output_osf_file = Path(sys.argv[2])

    # create folder to store the cache results
    curr_dir = Path.cwd()
    osf_dir = curr_dir / 'stitch_osf_cache'
    converted_osf_dir = osf_dir / 'osf'
    slammed_osf_dir = osf_dir / 'slam_processed_osf'
    converted_osf_results = converted_osf_dir / 'converted.osf'
    slamed_osf = slammed_osf_dir / 'processed.osf'

    # clean cache and avoid unexpected cache files merging
    delete_and_create_folder(osf_dir)
    delete_and_create_folder(converted_osf_dir)
    delete_and_create_folder(slammed_osf_dir)

    pcaps_to_osfs(input_path, osf_out_name=converted_osf_results)
    process_osf_files(converted_osf_dir, output=slamed_osf)
    stitch_osfs(slammed_osf_dir, output=output_osf_file)
