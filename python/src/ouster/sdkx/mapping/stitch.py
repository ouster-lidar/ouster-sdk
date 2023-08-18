import sys
import itertools
import os
import glob
import open3d
import math
import logging
import numpy as np
from pathlib import Path
from ouster.cli.core.pcap import pcap_slice_impl
import ouster.pcap as pcap
from ouster.client import _client
import ouster.osf as osf
import ouster.sdk.pose_util as pu
from ouster.sdk.util import resolve_metadata
from ouster import client
from ouster.sdkx.mapping import mapping
from typing import Deque, Union, cast
from collections import deque
from scapy.all import rdpcap, wrpcap

logger = logging.getLogger()
logging.basicConfig(level=logging.DEBUG)

# frame number per output osf
frames_per_out = 300
min_dist = 3  # meter
min_z = 2  # meter
max_z = 12  # meter

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
            if ranges[u][v] < min_dist:
                continue
            above = (max(0, u - 1), v)
            below = (min(scan.h - 1, u + 1), v)
            left = (u, (v - 1 + scan.w) % scan.w)
            right = (u, (v + 1) % scan.w)

            if (ranges[above[0]][above[1]] < min_dist or
                ranges[below[0]][below[1]] < min_dist or
                ranges[left[0]][left[1]] < min_dist or
                    ranges[right[0]][right[1]] < min_dist):
                continue
            diff_u = destagger_points[above[0]][above[1]] - destagger_points[below[0]][below[1]]
            diff_v = destagger_points[left[0]][left[1]] - destagger_points[right[0]][right[1]]
            normal = np.cross(diff_u, diff_v)
            n = np.sqrt(np.sum(normal ** 2))
            if n == 0:
                continue
            unit_normal = normal / n
            normals[u][v] = unit_normal

    stagger_normals = client.destagger(info, normals, True)
    return stagger_normals


def scans_to_cloud(scans: Deque[client.LidarScan], info, xyzlut, target_pose = np.eye(4)):
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
        flat_local_points = local_points.reshape(local_points.shape[0] * local_points.shape[1], 3)
        filter_idx = (flat_local_points[:, 2] >= min_z) & (flat_local_points[:, 2] <= max_z)
        global_points = pu.dewarp(local_points, scan_pose=target_pose, column_poses=scan.pose)
        flat_global_points = global_points.reshape(global_points.shape[0] * global_points.shape[1], 3)
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


def register_two_scans(target_scan, source_scan, xyzlut, target_pose, info, idx, debug = True) -> np.ndarray:
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
        logger.debug("transformation = {}".format(icp_result.transformation))
        open3d.io.write_point_cloud(
            "target_cloud_normal" +
            str(idx) +
            ".ply",
            target_cloud)
        open3d.io.write_point_cloud(
            "source_cloud_normal" +
            str(idx) +
            ".ply",
            source_cloud)
        open3d.io.write_point_cloud(
            "result_cloud_normal" +
            str(idx) +
            ".ply",
            result_cloud)

    return icp_result.transformation

# Have this function to avoid A010.ply < A02.ply case


def sort_key(file_name):
    name, ext = os.path.splitext(file_name)
    numeric_part = ''.join(filter(str.isdigit, name))
    return (name.replace(numeric_part, ''), int(numeric_part)) if numeric_part else (file_name, 0)


def stitch_osf(osf_path: PathSource, output: Output, meta: Meta):
    """
    Stitch OSF files into one.

    Use the last N scans and the first N scans to stitch clouds.
    This function works only for single lidar setups.

    Args:
        osf_path (Path, str): Path to the directory containing OSF files.
        output (Path, str): Path to the output file.
        meta (Path, str): Path to the meta file.
    """
    osf_path = Path(osf_path)
    output = Path(output)
    meta = str(meta)
    if osf_path.is_dir():
        osf_files = list(osf_path.glob("*.osf"))
    else:
        logger.error("Expect a directory which contains OSF files")
        return

    logger.info("Combining {} OSF files".format(len(osf_files)))

    # Sort the list of OSF files
    sorted_osf_files = sorted(osf_files, key=sort_key)

    # peak the first OSF file
    peak_reader = osf.Reader(str(sorted_osf_files[0]))
    sensor_meta = peak_reader.meta_store.get(osf.LidarSensor)
    del peak_reader
    info = sensor_meta.info
    xyzlut = client.XYZLut(info, use_extrinsics=True)

    osf_writer = osf.Writer(str(output))

    with open(meta) as meta_file:
        meta_json = meta_file.read()
        lidar_sensor_meta = osf.LidarSensor(meta_json)

    single_sensor_id = osf_writer.addMetadata(lidar_sensor_meta)
    lidar_stream = osf.LidarScanStream(osf_writer, single_sensor_id)

    # large max_scans could improve the stitch accuracy but increase the processing time
    max_scans = 5
    last_pose = np.eye(4)
    icp_diff = np.eye(4)
    first_scan = True
    last_scans_deque = deque(maxlen=max_scans)
    curr_scans_deque = deque(maxlen=max_scans)

    for file_id, file in enumerate(sorted_osf_files):
        logger.info("Processing #{} OSF file {}".format(file_id, file))
        reader = osf.Reader(str(file))
        for msg_id, msg in enumerate(reader.messages()):
            if msg.of(osf.LidarScanStream):
                # Skip the first scan from second OSF file as the first scan per
                # column poses are all eye(4) when SLAM restart. It will affect ICP matches
                if file_id != 0 and first_scan:
                    first_scan = False
                    continue

                obj = msg.decode()
                scan = cast(client.LidarScan, obj)
                if file_id != 0:
                    if len(curr_scans_deque) < curr_scans_deque.maxlen:
                        curr_scans_deque.append(scan)
                        continue

                    if np.all(icp_diff == np.eye(4)):
                        last_scan = last_scans_deque[-1]
                        last_pose = last_scan.pose[client.first_valid_column(last_scan)]
                        icp_diff = register_two_scans(last_scans_deque, curr_scans_deque,
                                                      xyzlut, last_pose, info, file_id, True)
                        for stored_scan in curr_scans_deque:
                            apply_icp(stored_scan, icp_diff, last_pose)
                            lidar_stream.save(client.first_valid_column_ts(stored_scan), stored_scan)
                            last_scans_deque.append(stored_scan)
                    apply_icp(scan, icp_diff, last_pose)

                lidar_stream.save(client.first_valid_column_ts(scan), scan)
                last_scans_deque.append(scan)

        icp_diff = np.eye(4)
        curr_scans_deque.clear()
        first_scan = True

    osf_writer.close()


def process_files(pcap_path: PathSource, meta: Meta,
                  output: Output):
    """
    Run slam on given PCAP files.
    This function works only for single lidar setups.

    Args:
        pcap_path (Path, str): Path of the directory containing pcap files.
        output (Path, str): Path to the output file.
        meta (Path, str): Path of the meta file.
    """
    pcap_path = Path(pcap_path)
    meta = str(meta)
    output = Path(output)
    if pcap_path.is_dir():
        pcap_files = list(pcap_path.glob("*.pcap"))
    elif pcap_path.is_file():
        pcap_files = [str(pcap_path)]
    else:
        logger.error("Input {} is neither a directory nor a file".format(pcap_path))
        return

    sorted_pcap_files = sorted(pcap_files, key=sort_key)
    logger.info("Start to process {} pcap files".format(len(pcap_files)))

    file_wo_ext = str(output.stem)
    for idx, pcap in enumerate(sorted_pcap_files):
        osf_output_file = os.path.join(output.parent, file_wo_ext + str(idx) + '.osf')
        mapping.run_slam_impl(pcap, meta=meta, output=osf_output_file)


def split_pcap(file_path: PathSource, meta: Meta,
               output: Output):
    """
    Split a large size pcap file into consecutive small pcap files.
    This function works only for single lidar setups.

    Args:
        pcap_path (Path, str): Path of a large size pcap file.
        output (Path, str): Path of the output file.
        meta (Path, str): Path of the meta file.
    """
    file_path = Path(file_path)
    meta = str(meta)
    output = Path(output)

    out_dir = output.parent
    file_wo_ext = output.stem
    outfile_ext = output.suffix
    total_packet = 0

    meta = resolve_metadata(file_path)
    pcap_all_info = pcap._packet_info_stream(str(file_path), 0)
    with open(meta) as json:
        info = client.SensorInfo(json.read())
        pf = client._client.PacketFormat.from_info(info)
        lp_size = pf.lidar_packet_size

        packet_per_frame = info.format.columns_per_frame / info.format.columns_per_packet

        for v in list(pcap_all_info.udp_streams.values()):
            # payload size count is (size, count)
            payload_size = list(v.payload_size_counts.items())[0][0]
            payload_count = list(v.payload_size_counts.items())[0][1]
            if payload_size == lp_size:
                total_packet = payload_count
        total_frame = math.ceil(total_packet / packet_per_frame)

    if total_frame < frames_per_out:
        import shutil
        output_file_path = str(out_dir / Path(file_wo_ext + str(0) + outfile_ext))
        logger.info("Source pcap file is too smal. Skip file split and copy it to {}".format(output_file_path))
        shutil.copy(str(file_path), output_file_path)
        return

    for idx, start_frame in enumerate(range(0, max(frames_per_out, total_frame) + 1, frames_per_out)):
        output_file_path = str(out_dir / Path(file_wo_ext + str(idx) + outfile_ext))
        logger.info("Slicing and saving into {}".format(output_file_path))
        pcap_slice_impl(
            str(file_path),
            start_frame=start_frame,
            num_frames=frames_per_out,
            meta=meta,
            lidar_port=None,
            imu_port=None,
            output=output_file_path,
            soft_id_check=True)


def combine_pcap_files(input_path: PathSource, output_file: str):
    """
    Combine all pcap files which are under the given path into a large one
    This function works only for single lidar setups.

    Args:
        pcap_path (Path, str): Path of a directory of pcap files.
        output (str): Path of the output file.
    """
    files_path = Path(input_path)
    if files_path.is_dir():
        pcap_files = list(files_path.glob("*.pcap"))

        if not pcap_files:
            logger.error("No PCAP files found in {}".format(pcap_path))
            return
    else:
        logger.error("Expect a directory which contains PCAP files")
        return

    sorted_pcap_files = sorted(pcap_files, key=sort_key)
    packets = []

    logger.info("Combining {} into {}".format(len(pcap_files), output_file))

    for input_file in sorted_pcap_files:
        packets.extend(rdpcap(str(input_file)))

    wrpcap(output_file, packets)


def delete_and_create_folder(folder_path):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
        logger.info("New folder {} created".format(folder_path))
    else:
        logger.warning("Folder '{}' exist. Be careful with the cache files".format(folder_path))


if __name__ == "__main__":
    if len(sys.argv) != 4:
        logger.error("Usage: python3 stitich.py PATH/data.pcap PATH/metadata.json PATH/output.osf \nExit")
        exit(1)

    logger.info("pcap file {}".format(sys.argv[1]))
    logger.info("meta file {}".format(sys.argv[2]))
    logger.info("output file {}".format(sys.argv[3]))

    # Tested this function but not use it here.
    # The purpose of the function is to combine multi recording into one pcap.
    # Then we use split_pcap function to seperate it.
    # combine_pcap_files(DIR, c.pcap)

    input_path = Path(sys.argv[1])
    meta_path = Path(sys.argv[2])
    output_path = Path(sys.argv[3])
    split_dir = input_path.parent / 'split'
    # clean cache and avoid unexpected pcap/osf merging
    delete_and_create_folder(split_dir)
    split_osf_results = split_dir / 'split.osf'
    split_pcap_results = split_dir / 'split.pcap'

    split_pcap(input_path, meta = meta_path, output=split_pcap_results)
    process_files(split_dir, meta = meta_path, output=split_osf_results)
    stitch_osf(split_dir, output = output_path, meta = meta_path)
