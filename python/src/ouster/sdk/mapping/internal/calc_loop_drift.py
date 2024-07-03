import os
import argparse
import logging
import open3d
import math
import numpy as np
from pathlib import Path
from datetime import datetime
from collections import deque
from ouster.sdk import open_source
import ouster.sdk.client as client
import ouster.sdk.util.pose_util as pu
from ouster.sdk.mapping.internal.stitch import scans_to_cloud

logger = logging.getLogger()
logging.basicConfig(level=logging.INFO)

# slam pose and icp pose difference threadhold
# if greater than the threadhold, the icp mostly likely has bad alignment
slam_icp_diff_thread = 3


def off_score(icp_diff_log):
    rot_weight = 0.5
    trans_weight = 0.5

    rot_off = math.sqrt((icp_diff_log[0])**2 +
                        (icp_diff_log[1])**2 +
                        (icp_diff_log[2])**2)

    trans_off = math.sqrt((icp_diff_log[3])**2 +
                          (icp_diff_log[4])**2 +
                          (icp_diff_log[5])**2)

    return rot_weight * rot_off + trans_weight * trans_off


def run(osf_file, debug):
    source = open_source(osf_file)
    info = source.metadata
    window = info.format.column_window
    xyzlut = client.XYZLut(info, use_extrinsics=True)

    first_scan = None
    last_scan = None
    for scan in source:
        if scan.complete(window):
            if not first_scan:
                first_scan = scan
            else:
                last_scan = scan

    if not first_scan or not last_scan:
        logger.error("Valid scans are less than 2, Exits")
        return

    logger.info(f"first scan ts = {first_scan.timestamp[client.first_valid_column(first_scan)]}")
    logger.info(f"last scan ts = {last_scan.timestamp[client.first_valid_column(last_scan)]}")

    first_scan_pose = first_scan.pose[client.first_valid_column(first_scan)]
    last_scan_pose = last_scan.pose[client.first_valid_column(last_scan)]
    first_cloud = scans_to_cloud(deque([first_scan]), info, xyzlut)
    last_cloud = scans_to_cloud(deque([last_scan]), info, xyzlut)

    max_correspondence_distance = 5
    criteria = open3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-12,
                                                                    relative_rmse=1e-12,
                                                                    max_iteration=2000)
    icp_result = open3d.pipelines.registration.registration_icp(
        first_cloud, last_cloud, max_correspondence_distance,
        estimation_method=open3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=criteria)

    slam_diff_h = np.matmul(np.linalg.inv(first_scan_pose), last_scan_pose)
    slam_diff_log = pu.log_pose(slam_diff_h)

    icp_diff_h = icp_result.transformation.copy()
    icp_diff_log = pu.log_pose(icp_diff_h)

    slam_icp_diff = slam_diff_log - icp_diff_log

    logger.info("ICP Pose Diff = %s", icp_diff_log)

    if debug:
        logger.info("SLAM Pose Diff = %s", slam_diff_log)
        logger.info("SLAM ICP Diff = %s", slam_icp_diff)

    slam_icp_off_score = off_score(slam_icp_diff)
    logger.info(f"ICP align mismatch: {slam_icp_off_score}")

    if slam_icp_off_score > slam_icp_diff_thread:
        logger.info("SLAM pose is far off than ICP alignment. ICP may mis-matched."
                    "Use --debug to view the point clouds. Or SLAM has huge drift.")

    # the smaller off the better slam result

    if debug:
        folder_name = "slam_eval_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs(Path(folder_name))
        open3d.io.write_point_cloud(folder_name + "/first_cloud.ply", first_cloud)
        open3d.io.write_point_cloud(folder_name + "/last_cloud.ply", last_cloud)
        aligned_first_cloud = first_cloud.transform(icp_diff_h)
        open3d.io.write_point_cloud(folder_name + "/aligned_first_cloud.ply", aligned_first_cloud)
        logger.info(f"Point Cloud Saved at {folder_name}. Expect the aligned_first_cloud "
                    f"perfectly matched with the last_cloud. Use cloudcompare to view")

    logger.info(" !! SLAM Start/End Poses Drift = %s", off_score(icp_diff_log))


def main():
    parser = argparse.ArgumentParser(description="Calculate loop type map drift.")
    parser.add_argument("slam_osf", help="slam output osf file")
    parser.add_argument('--debug', action='store_true', default=False,
                        help='debug point cloud saving and pose printout')

    args = parser.parse_args()

    run(args.slam_osf, args.debug)


if __name__ == "__main__":
    main()
