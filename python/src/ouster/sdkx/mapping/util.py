# type: ignore

from typing import Union, Tuple, Optional
from pathlib import Path
import numpy as np
from ouster import client, pcap
from ouster.sdk.util import resolve_metadata
import ouster.sdk.pose_util as pu
import ipaddress


"""
This function return the global_per_col_pose (4x4) of the input LidarScan by using
the constant pose change rate assumption. This function currently ONLY for KISS-ICP
Pose interpolaton.
KISS-ICP registers point cloud based on the middle ts, so thepose timestamp is
also based on middle ts of a scan.

scan:              |    start_pose  |    end_pose   |
ts (beginning):             |                |
                         start_ts         end_ts

ts (middle):       |                    output_scan
                      funct return-> global_col_poses
start_ts/end_ts are the middle vaild ts of the start_scan/end_scan.
* end_pose MUST BE the KISS-ICP output result of the output_scan
* for example: end_pose = kiss_icp.update(output_scan)
"""


def getScanColPose(start_ts_pose: Tuple[int, np.ndarray],
        end_ts_pose: Tuple[int, np.ndarray],
        scan: client.LidarScan) -> np.ndarray:
    start_pose_h = start_ts_pose[1]
    end_pose_h = end_ts_pose[1]
    diff_log = pu.log_pose(np.dot(end_pose_h, np.linalg.inv(start_pose_h)))

    start_ts = start_ts_pose[0]
    end_ts = end_ts_pose[0]
    delta_ts = end_ts - start_ts

    if delta_ts < 0:
        raise RuntimeError(
            "end pose ts smaller then start pose ts.\nexit"
        )

    # As end_ts_pose is the kiss-icp results of the scan. end_ts will be the
    # middle ts of the scan. Use end_ts as mid_scan_ts to avoid calculation
    per_col_diff_v = [
        ((ts - end_ts) / delta_ts) * diff_log for ts in scan.timestamp
    ]

    per_col_diff_h = pu.exp_pose6(np.array(per_col_diff_v))

    start_pose_h = start_pose_h[np.newaxis, :, :]
    end_pose_h = end_pose_h[np.newaxis, :, :]
    global_col_poses = np.einsum("ijk,ikl->ijl", per_col_diff_h, end_pose_h)

    return global_col_poses


class MetadataNotFound(Exception):
    pass


def _get_pcap_metadata(source_location, meta_location):
    meta_data = resolve_metadata(source_location, meta_location)
    if not meta_data:
        raise MetadataNotFound(
            "Metadata not found. Check they are present or explcitly pass a path to them."
        )
    with open(meta_data, 'r') as meta_file:
        meta_json = meta_file.read()
        info = client.SensorInfo(meta_json)
    return info


def Source(
    source_location: Union[str, Path, client.Scans],
    *,
    meta: Optional[Union[str, Path]] = None,
    lidar_port: int = 7502,
    imu_port: int = 7503,
) -> client.Scans:
    # If we already have scans just return them back
    if isinstance(source_location, client.Scans):
        return source_location

    # We do not have return source. Let's get it
    # We run str on paths because they might be Path instances
    source_location = str(source_location)
    source_ext = Path(source_location).suffix
    meta_location = meta and str(meta)

    if source_ext == ".pcap":
        info = _get_pcap_metadata(source_location, meta_location)
        # hard-coded variable for process status printout #

        # reopen pcap file as the old iterator point to the end
        pcap_source = pcap.Pcap(
            str(source_location), info, lidar_port=lidar_port, imu_port=imu_port
        )
        return client.Scans(pcap_source, timeout=None, complete=False, _max_latency=2)
    elif source_ext == ".local" or ipaddress.ip_address(source_location):
        sensor_source = client.Sensor(str(source_location), lidar_port, imu_port)
        return client.Scans(sensor_source, timeout=None, complete=False, _max_latency=2)
    else:
        raise Exception(
            "The library only supports running with a live sensor or a recorded pcap")
