"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.
"""

from typing import TYPE_CHECKING, Tuple, List, Callable, Union

import os
import pytest
import numpy as np

import math

from ouster import client
import ouster.pcap as pcap
from ouster.sdk.util import resolve_metadata
import ouster.sdk.pose_util as pu

from ouster.viz import grey_palette
from ouster.viz.scans_accum import ScansAccumulator

try:
    from scipy.spatial.transform import Rotation as R
    _no_scipy = False
except ImportError:
    _no_scipy = True

from .test_viz import point_viz as point_viz_plain  # noqa: F401

# test env may not have opengl, but all test modules are imported during
# collection. Import is still needed to typecheck
if TYPE_CHECKING:
    import ouster.viz as viz
else:
    viz = pytest.importorskip('ouster.viz')

# Loose vector type
Vector = Union[List, Tuple, np.ndarray]

# mark all tests in this module so they only run with the --interactive flag
pytestmark = pytest.mark.interactive


@pytest.fixture
def point_viz(point_viz_plain: viz.PointViz) -> viz.PointViz:  # noqa: F811
    # Some better focused view
    point_viz_plain.camera.set_yaw(140)
    point_viz_plain.camera.set_pitch(-75)
    point_viz_plain.camera.set_dolly(-170)
    point_viz_plain.target_display.set_ring_size(0)
    point_viz_plain.target_display.enable_rings(True)
    return point_viz_plain


def pose_from_tr(translation: Vector, rotation: Vector) -> np.ndarray:
    """Construct homogeneous 4x4 transform from translation/rotation parts.

    Args:
      translation: [3,1] vector in meters
      rotation: [3,1] rotation vector in degrees
    """
    p = np.eye(4)
    p[:3, 3] = np.asarray(translation)
    p[:3, :3] = R.from_euler('XYZ', np.asarray(rotation),
                             degrees=True).as_matrix()
    return p


def spin(pviz: viz.PointViz,
         on_update: Callable[[viz.PointViz, float], None],
         *,
         period: float = 0.0333,
         total: float = 0,
         title: str = "") -> None:
    import threading
    import time

    title_label = viz.Label(title, 0, 1)
    pviz.add(title_label)

    quit = threading.Event()

    def animate() -> None:
        first_tick_ts = 0.0
        last_tick_ts = 0.0
        while not quit.is_set():
            dt = time.monotonic() - last_tick_ts
            time.sleep(period - dt if period - dt > 0 else 0)
            last_tick_ts = time.monotonic()
            if not first_tick_ts:
                first_tick_ts = last_tick_ts
            on_update(pviz, last_tick_ts - first_tick_ts)
            pviz.update()

            if total > 0 and time.monotonic() - first_tick_ts > total:
                break

    thread = threading.Thread(target=animate)
    thread.start()

    pviz.run()
    quit.set()
    thread.join()


def test_viz_util_axes(point_viz: viz.PointViz) -> None:
    """Test displaying a full-window image."""

    viz.AxisWithLabel(point_viz,
                      label="O",
                      thickness=3,
                      label_scale=1,
                      enabled=True)

    point_viz.update()
    point_viz.run()


def test_viz_util_spin(point_viz: viz.PointViz) -> None:
    """Test displaying a spin and axis movement."""
    axis = viz.AxisWithLabel(point_viz,
                             label="O",
                             thickness=3,
                             label_scale=1,
                             enabled=True)

    def on_update(_, tick_ts) -> None:
        axis.pose[0, 3] += 0.1
        axis.label = f"{tick_ts:.03f}"
        axis.update()

    spin(point_viz, on_update, period=0.1, total=2, title="Spin and Axis move")


@pytest.mark.skipif(_no_scipy, reason="didn't have the scipy.")
def test_viz_util_one_arc(point_viz: viz.PointViz) -> None:
    """Test moving along the arc."""
    axis_a = viz.AxisWithLabel(point_viz,
                               label="A",
                               thickness=5,
                               length=1.0,
                               label_scale=0.3)

    axis_b = viz.AxisWithLabel(point_viz,
                               label="B",
                               thickness=5,
                               length=0.5,
                               label_scale=0.3)

    viz.AxisWithLabel(point_viz,
                      pose=pose_from_tr([0, 0, 0], [0, 0, 0]),
                      label="O",
                      thickness=10,
                      length=0.3,
                      enabled=True)

    point_z = viz.AxisWithLabel(point_viz,
                                pose=pose_from_tr([0, 0, 3], [90, 0, 0]),
                                label="X",
                                thickness=10,
                                length=0.1,
                                enabled=True)

    point_y = viz.AxisWithLabel(point_viz,
                                pose=pose_from_tr([0, 3, 0], [90, 90, 0]),
                                label="Y",
                                length=0.1,
                                thickness=10,
                                enabled=True)

    period_t = 0.1
    total_t = 2

    pose_from = point_y.pose.copy()
    pose_to = point_z.pose.copy()

    rot_delta_vec = R.from_matrix(
        np.linalg.inv(pose_from[:3, :3]) @ pose_to[:3, :3]).as_rotvec()

    pose_delta = pu.log_pose(np.linalg.inv(pose_from) @ pose_to)

    axis_a.pose = pose_from.copy()
    axis_b.pose = pose_from.copy()

    def on_update(_, tick_ts) -> None:
        t = 1.5 * (tick_ts / total_t) - 0.25

        # B: Slerp + Linear interpolation
        axis_b.pose[:3, :3] = pose_from[:3, :3] @ R.from_rotvec(rot_delta_vec * t).as_matrix()
        axis_b.pose[:3, 3] = pose_from[:3, 3] + (pose_to[:3, 3] - pose_from[:3, 3]) * t
        axis_b.update()

        # A: SE3 manifold interpolation
        axis_a.pose = pose_from @ pu.exp_pose6(pose_delta * t)
        axis_a.update()

        # print("delta_translation = ", np.linalg.norm((axis_a.pose - axis_b.pose)[:3, 3]))

    spin(point_viz,
         on_update,
         period=period_t,
         total=total_t,
         title="Interpolation: Slerp + Linear VS SE3/geodesical")


def test_viz_util_traj_eval(point_viz: viz.PointViz) -> None:
    """Test moving along the trajectory path."""
    poses6 = [
        np.array([0, 0, 0, 1, 0, 0]),
        np.array([math.pi / 2, 0, 0, 0, 1, 0]),
        np.array([0, - math.pi / 2, 0, 0, 0, 1]),
        np.array([0, 0, 0, 1, 0, 0]),
    ]

    # time dilation compare to real time if poses changed every 1 sec
    time_dil = 3

    traj_poses = list([(i * time_dil, p) for i, p in enumerate(poses6)])
    traj_eval = pu.TrajectoryEvaluator(traj_poses)

    axis_a = viz.AxisWithLabel(point_viz,
                               pose=traj_eval.pose_at(0),
                               label="A",
                               thickness=5,
                               length=1.0,
                               label_scale=0.3)

    viz.AxisWithLabel(point_viz,
                      pose=np.eye(4),
                      label="O",
                      thickness=10,
                      length=0.3,
                      enabled=True)

    for i, p in enumerate(poses6):
        viz.AxisWithLabel(point_viz,
                          pose=pu.exp_pose6(p),
                          label=f"P{i}",
                          thickness=10,
                          length=0.1,
                          enabled=True)

    period_t = 0.1
    total_t = 3 * time_dil * len(poses6)

    def on_update(_, tick_ts) -> None:
        t = tick_ts % ((len(poses6) - 1) * time_dil)
        # A: move along the trajectory
        axis_a.pose = traj_eval.pose_at(t)
        axis_a.update()

    spin(point_viz,
         on_update,
         period=period_t,
         total=total_t,
         title="Trajectory Interpolation: move along the path")


@pytest.fixture(params=[
    "OS-1-128_v2.3.0_1024x10_lb_n3_poses_kitti.txt",
    "OS-1-128_v3.0.1_1024x10_20230216_142857_poses_kitti.txt"
])
def kitti_poses_file(test_data_dir, request) -> str:
    return str(test_data_dir / "pcaps" / request.param)


def test_viz_util_traj_eval_kitti(kitti_poses_file, point_viz: viz.PointViz) -> None:
    """Test moving along the trajectory path from kitti poses."""
    poses = pu.load_kitti_poses(kitti_poses_file)

    point_viz.camera.set_dolly(-400)

    traj_poses = pu.make_kiss_traj_poses(poses)
    traj_eval = pu.TrajectoryEvaluator(traj_poses, time_bounds=1.0)

    axis_a = viz.AxisWithLabel(point_viz,
                               pose=traj_eval.pose_at(0),
                               label="A",
                               thickness=5,
                               length=0.2,
                               label_scale=0.3)

    viz.AxisWithLabel(point_viz,
                      pose=np.eye(4),
                      label="O",
                      thickness=10,
                      length=0.1,
                      label_scale=0.3,
                      enabled=True)

    for i, p in enumerate(poses):
        viz.AxisWithLabel(point_viz,
                          pose=p,
                          thickness=10,
                          length=0.1,
                          label_scale=0.3,
                          enabled=True)

    loops = 1
    period_t = 0.01
    total_traj_t = traj_poses[-1][0] + 0.5
    total_t = loops * total_traj_t

    def on_update(pviz, tick_ts) -> None:
        t = tick_ts % total_traj_t
        # A: move along the trajectory
        axis_a.pose = traj_eval.pose_at(t)
        axis_a.update()

        # update camera position to move along
        pviz.camera.set_target(np.linalg.inv(axis_a.pose))

    poses_filename = os.path.basename(kitti_poses_file).replace("_", "..")
    spin(
        point_viz,
        on_update,
        period=period_t,
        total=total_t,
        title=f"Trajectory Interpolation: move along the path (kitti poses):\n"
        f"{poses_filename}")


@pytest.mark.parametrize("use_dewarp", [True, False])
def test_viz_util_traj_eval_scans_poses(test_data_dir,
                                        point_viz: viz.PointViz,
                                        use_dewarp: bool) -> None:
    """Test to draw 3 scans with poses."""
    pcap_file = str(test_data_dir / "pcaps" /
                    "OS-1-128_v2.3.0_1024x10_lb_n3.pcap")

    poses_file = str(test_data_dir / "pcaps" /
                     "OS-1-128_v2.3.0_1024x10_lb_n3_poses_kitti.txt")

    # load one pose per scan
    poses = pu.load_kitti_poses(poses_file)

    # make time indexed poses starting from 0.5
    traj_poses = pu.make_kiss_traj_poses(poses)
    traj_eval = pu.TrajectoryEvaluator(traj_poses, time_bounds=1.0)

    meta = client.SensorInfo(open(resolve_metadata(pcap_file) or '').read())
    packets = pcap.Pcap(pcap_file, meta)
    scans = client.Scans(packets)

    # used for use_dewarp option
    xyzlut = client.XYZLut(meta)

    for idx, scan in enumerate(scans):
        # make scan indexed column timestamps
        idx_ts = idx + np.linspace(0, 1.0, scan.w, endpoint=False)
        traj_eval(scan, col_ts=idx_ts)
        key = scan.field(client.ChanField.REFLECTIVITY)
        key = key / np.amax(key)
        if use_dewarp:
            cloud_scan = viz.Cloud(scan.h * scan.w)
            xyz = xyzlut(scan.field(client.ChanField.RANGE))
            xyz = pu.dewarp(xyz, column_poses=scan.pose)
            cloud_scan.set_xyz(xyz)
            cloud_scan.set_key(key)
        else:
            cloud_scan = viz.Cloud(meta)
            cloud_scan.set_range(scan.field(client.ChanField.RANGE))
            cloud_scan.set_column_poses(scan.pose)
            cloud_scan.set_key(key)
        point_viz.add(cloud_scan)

    point_viz.update()

    viz.AxisWithLabel(point_viz,
                      pose=np.eye(4),
                      label="O",
                      thickness=5,
                      length=1,
                      label_scale=1,
                      enabled=True)

    for i, p in enumerate(poses):
        viz.AxisWithLabel(point_viz,
                          pose=p,
                          label=f"{i}",
                          thickness=2,
                          length=0.3,
                          label_scale=0.3,
                          enabled=True)

    total_traj_t = 2

    # some camera movement along 3 scan point cloud
    cam_traj_eval = pu.TrajectoryEvaluator([
        (0, np.array([0, 0, 0, -15, 0, 0])),
        (total_traj_t, np.array([0, 0, - math.pi / 12, 1, 0, 0]))
    ])

    point_viz.target_display.enable_rings(False)

    period_t = 0.01
    total_t = 0.95 * total_traj_t  # stop a little bit early

    def on_update(pviz, tick_ts) -> None:
        t = tick_ts % total_traj_t
        # add some camera movement
        pviz.camera.set_target(np.linalg.inv(cam_traj_eval.pose_at(t)))

    spin(point_viz,
         on_update,
         period=period_t,
         total=total_t,
         title=f"Trajectory Interpolation: move along the path (kitti poses) "
         f"with scans. Dewarp: {'YES' if use_dewarp else 'NO'}")


def test_viz_util_scans_accum_poses(test_data_dir,
                                    point_viz: viz.PointViz) -> None:
    """Test to draw 3 scans with poses using ScansAccumulator."""
    pcap_file = str(test_data_dir / "pcaps" /
                    "OS-1-128_v2.3.0_1024x10_lb_n3.pcap")

    poses_file = str(test_data_dir / "pcaps" /
                     "OS-1-128_v2.3.0_1024x10_lb_n3_poses_kitti.txt")

    meta = client.SensorInfo(open(resolve_metadata(pcap_file) or '').read())
    packets = pcap.Pcap(pcap_file, meta)
    scans = client.Scans(packets)
    scans_w_poses = pu.pose_scans_from_kitti(scans, poses_file)

    viz.AxisWithLabel(point_viz,
                      pose=np.eye(4),
                      label="O",
                      thickness=5,
                      length=1,
                      label_scale=1,
                      enabled=True)

    scans_acc = ScansAccumulator(meta,
                                 point_viz=point_viz,
                                 accum_max_num=10,
                                 accum_min_dist_num=1,
                                 map_enabled=True,
                                 map_select_ratio=0.5)

    for scan in scans_w_poses:
        scans_acc.update(scan)

    scans_acc.draw(update=True)

    total_traj_t = 2

    # some camera movement along 3 scan point cloud
    cam_traj_eval = pu.TrajectoryEvaluator([
        (0, np.array([0, 0, 0, -15, 0, 0])),
        (total_traj_t, np.array([0, 0, - math.pi / 12, 1, 0, 0]))
    ])

    point_viz.target_display.enable_rings(False)

    period_t = 0.01
    total_t = 0.95 * total_traj_t  # stop a little bit early

    def on_update(pviz, tick_ts) -> None:
        t = tick_ts % total_traj_t
        # add some camera movement
        pviz.camera.set_target(np.linalg.inv(cam_traj_eval.pose_at(t)))

    spin(point_viz,
         on_update,
         period=period_t,
         total=total_t,
         title="ScansAccumulator as scan viz.")


def test_viz_util_scans_accum_no_viz(test_data_dir,
                                     point_viz: viz.PointViz) -> None:
    """Test to draw 3 scans with poses using ScansAccumulator (without viz)"""
    pcap_file = str(test_data_dir / "pcaps" /
                    "OS-1-128_v2.3.0_1024x10_lb_n3.pcap")

    poses_file = str(test_data_dir / "pcaps" /
                     "OS-1-128_v2.3.0_1024x10_lb_n3_poses_kitti.txt")

    meta = client.SensorInfo(open(resolve_metadata(pcap_file) or '').read())
    packets = pcap.Pcap(pcap_file, meta)
    scans = client.Scans(packets)
    scans_w_poses = pu.pose_scans_from_kitti(scans, poses_file)

    viz.AxisWithLabel(point_viz,
                      pose=np.eye(4),
                      label="O",
                      thickness=5,
                      length=1,
                      label_scale=1,
                      enabled=True)

    # create scans accum without PointViz
    scans_acc = ScansAccumulator(meta,
                                 map_enabled=True,
                                 map_select_ratio=0.5)

    # processing doesn't require viz presence in scans accum
    for scan in scans_w_poses:
        scans_acc.update(scan)

    # draw the cloud manually to the viz using ScansAccumulator MAP data
    # TODO[pb]: make the real interfaces to get the data from
    #           ScansAccumulator back instead of tapping into
    #           internals.
    cloud_map = viz.Cloud(scans_acc._map_xyz.shape[0])
    cloud_map.set_xyz(scans_acc._map_xyz)
    cloud_map.set_key(scans_acc._map_keys["NEAR_IR"])
    cloud_map.set_palette(grey_palette)
    cloud_map.set_point_size(1)
    point_viz.add(cloud_map)

    total_traj_t = 2

    # some camera movement along 3 scan point cloud
    cam_traj_eval = pu.TrajectoryEvaluator([
        (0, np.array([0, 0, 0, -15, 0, 0])),
        (total_traj_t, np.array([0, 0, - math.pi / 12, 1, 0, 0]))
    ])

    point_viz.target_display.enable_rings(False)

    period_t = 0.01
    total_t = 0.95 * total_traj_t  # stop a little bit early

    def on_update(pviz, tick_ts) -> None:
        t = tick_ts % total_traj_t
        # add some camera movement
        pviz.camera.set_target(np.linalg.inv(cam_traj_eval.pose_at(t)))

    spin(point_viz,
         on_update,
         period=period_t,
         total=total_t,
         title="ScansAccumulator without viz, draw map manually")
