import sys
import logging
import numpy as np
from typing import List, Optional

import ouster.sdk.mapping.ouster_kiss_icp as ouster_kiss_icp
import ouster.sdk.mapping.util as util
import ouster.sdk.client as client

from .slam_backend import SLAMBackend

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger()


class KissBackend(SLAMBackend):
    """Wraps kiss-icp odometry to use with Ouster pipelines."""

    def __init__(
        self,
        infos: List[client.SensorInfo],
        use_extrinsics: bool = True,
        max_range: float = 150.0,
        min_range: float = 1.0,
        voxel_size: float = -1.0,
        live_stream: bool = False
    ):
        try:
            from kiss_icp.kiss_icp import KissICP  # type: ignore  # noqa
        except ImportError:
            logger.error("kiss-icp, a package required for slam, is unsupported on "
                         "your platform.")
            raise
        import kiss_icp.config  # type: ignore  # noqa
        assert infos, "infos should contain at least one SensorInfo"
        super().__init__(infos, use_extrinsics)
        # to store the middle valid timestamp of a scan and middle col pose
        self.last_slam_pose = None
        self.last_frame_id = [-1] * len(infos)
        self.config = kiss_icp.config.KISSConfig(None)
        self.config.data.deskew = True
        self.config.data.max_range = max_range
        self.config.data.min_range = min_range
        self.voxel_size = voxel_size
        self.live_stream = live_stream
        self.scans_ts_offset: List[int] = []
        if voxel_size and voxel_size > 0:
            self.config.mapping.voxel_size = voxel_size
            logger.info(f"Kiss-ICP voxel map size is {voxel_size:.4g} m")
            self._config_kiss_icp(self.config)
        self.max_frame_id = sys.maxsize
        for info in infos:
            self.max_frame_id = min(self.max_frame_id,
                                    client.PacketFormat(info).max_frame_id)

    def _config_kiss_icp(self, config):
        self.ouster_kiss_icp = ouster_kiss_icp.KissICP(config)

    def update(self, scans: List[Optional[client.LidarScan]]) -> List[Optional[client.LidarScan]]:
        """Update the pose (per_column_global_pose) variable in each scan"""
        assert scans

        if not self.scans_ts_offset:
            self.scans_ts_offset = self._get_scans_ts_offset(scans)
            if not self.scans_ts_offset:
                return scans

        if not self.voxel_size or self.voxel_size <= 0:
            self.voxel_size = self._get_voxel_size(scans)
            self.config.mapping.voxel_size = self.voxel_size
            self._config_kiss_icp(self.config)
            logger.info(f"Auto voxel size calculated based on the first scan "
                        f"which is {self.voxel_size:.4g} m.")

        # if frames drop, use it properly calculate the pose
        slam_update_diff = sys.maxsize
        for idx, scan in enumerate(scans):
            if not scan:
                continue
            curr_frame_id = scan.frame_id
            if self.last_frame_id[idx] == -1:
                frame_id_diff = 1
            else:
                frame_id_diff = curr_frame_id - self.last_frame_id[idx]

            # magic number -65530 is for 16 bit int overflow case and considered the
            # potential missing scans cases.
            OVERFLOW_PROTECTION_THRESHOLD = -65500
            if self.last_frame_id[idx] != -1 and curr_frame_id <= self.last_frame_id[idx] \
                    and frame_id_diff > OVERFLOW_PROTECTION_THRESHOLD:
                logger.warning("Set the out of order scan invalid")
                scan.field(client.ChanField.RANGE)[:] = 0

            # Prevent integer overflow
            frame_id_diff = frame_id_diff % self.max_frame_id
            slam_update_diff = min(frame_id_diff, slam_update_diff)

            self.last_frame_id[idx] = curr_frame_id

        icp_inputs = util.getKissICPInputData(scans, self.xyz_lut, self.scans_ts_offset)

        if icp_inputs[1].size == 0:
            # empty kiss icp intput mean the scans list has no valid scan
            return scans

        try:
            self.ouster_kiss_icp.register_frame(icp_inputs[0], icp_inputs[1],
                                                slam_update_diff)
        except Exception as e:
            from kiss_icp import __version__ as kiss_icp_version

            logger.error(f"KISS-ICP {kiss_icp_version} is incompatible with "
                         f"ouster-sdk\n"
                         f"Error message: {e}")
            raise

        # Use SLAM pose to update scans' col poses
        util.writeScanColPose(self.last_slam_pose, self.ouster_kiss_icp.last_pose,
                              scans, icp_inputs[2], slam_update_diff)

        self.last_slam_pose = self.ouster_kiss_icp.last_pose

        # retuen the updated col pose scans
        return scans

    @staticmethod
    def _get_scans_ts_offset(scans: List[Optional[client.LidarScan]]) -> List[int]:
        """Calculate the timestamp offsets for a list of LidarScans relative to the
           smallest packet timestamp."""

        def calculate_packet_ts_offsets(scans: List[Optional[client.LidarScan]]) -> List[int]:
            pkt_ts_offset = [int(client.first_valid_packet_ts(scan)) for scan in scans if scan]

            # FIXME[tws] handle ValueError if pkt_ts_offset is empty
            min_pkt_ts = min(pkt_ts_offset)
            return [ts - min_pkt_ts for ts in pkt_ts_offset]

        def calculate_scan_ts_offsets(scans: List[Optional[client.LidarScan]]) -> List[int]:
            return [int(client.first_valid_column_ts(scan)) for scan in scans if scan]

        # Validate scans input
        if not scans or any(scan is None for scan in scans):
            logger.warning("One or all scans list is None. Use the next scans to calculate offsets.")
            return []

        # Get packet and scan timestamp offsets
        packet_ts_offset = calculate_packet_ts_offsets(scans)
        scan_ts_offset = calculate_scan_ts_offsets(scans)

        total_offset = [pkt_ts - scan_ts for pkt_ts, scan_ts in zip(packet_ts_offset,
                                                                    scan_ts_offset)]

        return total_offset

    def _get_voxel_size(self, scans: List[Optional[client.LidarScan]], start_pct: float = 0.92,
                       end_pct: float = 0.96) -> float:
        """Average highest 92% to 96% range readings and use this averaged range value
            to calculate the voxel map size """
        selected_ranges = np.array([])

        for idx, scan in enumerate(scans):
            if not scan:
                continue
            sel_flag = scan.field(client.ChanField.RANGE) != 0
            scan_range = scan.field(client.ChanField.RANGE)[sel_flag]
            start_value = np.percentile(scan_range, start_pct * 100)
            end_value = np.percentile(scan_range, end_pct * 100)

            selected_range = scan_range[(scan_range >= start_value) & (scan_range <= end_value)]
            selected_ranges = np.concatenate((selected_ranges, selected_range))

        # lidar range is in mm. change the unit to meter
        average = np.mean(selected_ranges) / 1000
        # use the lidar range readings and a number to land voxel size in a
        # proper range
        voxel_size = average / 46.0

        # For live sensor, we use a larger voxel size for better results
        live_voxel_ratio = 2.2
        if self.live_stream:
            voxel_size *= live_voxel_ratio
            logger.warning("Auto voxel sizer uses a larger size voxel for the live "
                           "sensor real-time processing.")

        return voxel_size
