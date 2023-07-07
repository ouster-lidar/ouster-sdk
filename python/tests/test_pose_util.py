from typing import List

import math

import numpy as np

import time
import pytest


import ouster.sdk.pose_util as pu


def gt_pose6toHomMatrix(vec: np.ndarray) -> np.ndarray:
    """Convert exponential pose of [6] vector to homogeneous matrix [4,4]."""
    return pu._no_scipy_exp_pose6(vec)


# Lynch and Park: 3.3.3.2, Page 106, Algorithm
def gt_homMatToPose6(hmat):
    """Convert homogeneous matrix [4,4] to exp pose coordinates."""
    return pu._no_scipy_log_pose(hmat)


@pytest.fixture
def poses6() -> List[pu.Pose6]:
    # Some better focused view
    return [
        np.array([0, 0, 0, 0, 0, 0]),
        np.array([0, 0, 0, 1, 0, 0]),
        np.array([0, 0, 0, 0, 1, 0]),
        np.array([0, 0, 0, 0, 0, 1]),
        np.array([math.pi / 2, 0, 0, 0, 0, 0]),
        np.array([-math.pi / 2, 0, 0, 0, 0, 0]),
        np.array([0, math.pi / 2, 0, 0, 0, 0]),
        np.array([0, -math.pi / 2, 0, 0, 0, 0]),
        np.array([0, 0, math.pi / 2, 0, 0, 0]),
        np.array([0, 0, -math.pi / 2, 0, 0, 0]),
        np.array([math.pi / 2, 0, 0, 1, 0, 0]),
        np.array([-math.pi / 2, 0, 0, 1, 0, 0]),
        np.array([0, math.pi / 2, 0, 0, 1, 0]),
        np.array([0, -math.pi / 2, 0, 0, 1, 0]),
        np.array([0, 0, math.pi / 2, 0, 0, 1]),
        np.array([0, 0, 0, 0, 0, 1]),
    ]


def test_exp_log_pose_vectorized(poses6: List[pu.Pose6]):
    # homogeneous ground truth poses, [N, 4, 4] size
    gt_p_vec = []
    for p in poses6:
        gt_p_vec.append(gt_pose6toHomMatrix(p))
        # test one by one conversions of vectors
        exp_p = pu.exp_pose6(p)
        assert np.allclose(gt_p_vec[-1], exp_p)
        # check roundtrip vectorized, one vec
        assert np.allclose(p, pu.log_pose(exp_p))
        # check roundtrip p -> gt -> p
        assert np.allclose(p, gt_homMatToPose6(exp_p))

    # check vectrised conversion of [N, 6] size
    vec_p = pu.exp_pose6(np.array(poses6))
    assert np.allclose(np.array(gt_p_vec), vec_p)

    # check roundtrip vectorized of [N, 4, 4] size
    assert np.allclose(poses6, pu.log_pose(vec_p))


def test_traj_pose_interp(poses6: List[pu.Pose6]):
    # homogeneous ground truth poses, [N, 4, 4] size
    num_poses = len(poses6)
    poses1 = []
    ts = np.linspace(0,
                     num_poses - 1,
                     num=np.random.randint(20 * num_poses, 200 * num_poses),
                     endpoint=True)
    t0 = time.monotonic()
    for idx, t in enumerate(ts):
        p_low = int(np.floor(t))
        p_high = int(np.ceil(t))
        dt = t - p_low
        p_t = pu.pose_interp(poses6[p_low], poses6[p_high], dt)
        poses1.append(p_t)
    t_pose_interp = time.monotonic() - t0

    # use traj interpolation
    traj_poses = list(zip(range(num_poses), poses6))
    traj_eval = pu.TrajectoryEvaluator(traj_poses)

    poses2 = []
    t0 = time.monotonic()
    # one by one interpolation
    for idx, t in enumerate(ts):
        p_t = traj_eval.pose_at(t)
        poses2.append(p_t)
    t_pose_te_one_by_one = time.monotonic() - t0

    assert np.allclose(np.array(poses1), np.array(poses2))

    t0 = time.monotonic()
    # all vectorised one interpolation
    poses3 = traj_eval.poses_at(ts)
    t_pose_te_vec = time.monotonic() - t0
    assert np.allclose(np.array(poses1), poses3)

    t0 = time.monotonic()
    # all vectorised interpolation in one func
    poses4 = pu.traj_interp(traj_poses, ts)
    t_pose_traj_interp = time.monotonic() - t0
    assert np.allclose(np.array(poses1), poses4)

    print(f"Summary times for {len(ts)} poses calc:")
    print(f"  pose_interp() ........ one by one :  {t_pose_interp:.08f} s")
    print(f"  traj_eval.pose_at() .. one by one :  {t_pose_te_one_by_one:.08f} s "
          f"({t_pose_interp/t_pose_te_one_by_one:.02f}x)")
    vec_speedup = ""
    if t_pose_te_vec > 0:
        vec_speedup = f"({t_pose_te_one_by_one/t_pose_te_vec:.02f}x)"
    else:
        vec_speedup = "(FTL)"  # Faster Than Light
    print(f"  traj_eval.poses_at() . vectorised :  {t_pose_te_vec:.08f} s "
          f"{vec_speedup}")
    print(f"  traj_interp(), helper  vectorised :  {t_pose_traj_interp:.08f} s")


def test_load_kitti_poses(test_data_dir):
    kitti_poses_file = str(test_data_dir / "pcaps" /
                           "OS-1-128_v2.3.0_1024x10_lb_n3_poses_kitti.txt")
    poses = pu.load_kitti_poses(kitti_poses_file)
    assert len(poses) == 3
    assert np.allclose(poses, pu.exp_pose6(pu.log_pose(poses)))


def test_traj_eval_bounds(poses6: List[pu.Pose6]):
    traj_poses = list([(0.5 + i, p) for i, p in enumerate(poses6)])

    # no bounds set, so any ts should evaluate to smth
    te = pu.TrajectoryEvaluator(traj_poses, time_bounds=None)

    p_0 = te.pose_at(0.0)
    # mid between poses6[1] and poses6[0], moving left
    assert np.allclose(p_0, pu.exp_pose6(np.array([0, 0, 0, -0.5, 0, 0])))

    p_len = te.pose_at(len(poses6))
    assert np.allclose(p_len, pu.exp_pose6(np.array([0, 0, - math.pi / 4, 0, 0, 1])))

    p_05 = te.pose_at(0.5)
    assert np.allclose(p_05, pu.exp_pose6(poses6[0]))

    # 0.25 move between poses6[0] and poses6[1]
    p_075 = te.pose_at(0.75)
    assert np.allclose(p_075, pu.exp_pose6(np.array([0, 0, 0, 0.25, 0, 0])))

    p_last = te.pose_at(len(poses6) - 0.5)
    assert np.allclose(p_last, pu.exp_pose6(poses6[-1]))

    ts = np.array([0.0, 0.5, 0.75, len(poses6) - 0.5, len(poses6)])
    ts_poses_gt = np.array([p_0, p_05, p_075, p_last, p_len])

    # 0.5, 1.0 bounds set, just to cover +/-0.5 out of bounds traj requests
    for tb in [0.5, 1.0, 10.0]:
        te = pu.TrajectoryEvaluator(traj_poses, time_bounds=tb)
        assert np.allclose(te.pose_at(0.0), p_0)
        assert np.allclose(te.pose_at(len(poses6)), p_len)
        assert np.allclose(te.pose_at(0.5), p_05)
        assert np.allclose(te.pose_at(0.75), p_075)
        assert np.allclose(te.pose_at(len(poses6) - 0.5), p_last)

        assert np.allclose(te.poses_at(ts), ts_poses_gt)

    # 0 or 0.4 bounds set, NOT enough to cover +/-0.5 out of bounds traj requests
    for tb in [0, 0.4]:
        te = pu.TrajectoryEvaluator(traj_poses, time_bounds=tb)
        for t in [0.0, len(poses6)]:
            with pytest.raises(ValueError):
                te.pose_at(t)
        assert np.allclose(te.pose_at(0.5), p_05)
        assert np.allclose(te.pose_at(0.75), p_075)
        assert np.allclose(te.pose_at(len(poses6) - 0.5), p_last)

        # some elements of ts vec is out of bounds
        with pytest.raises(ValueError):
            te.poses_at(ts)

        assert np.allclose(
            te.poses_at(np.array([0.5, 0.75, len(poses6) - 0.5])),
            np.array([p_05, p_075, p_last]))

    # -0.1 bounds set, NOT enough to cover left/right edges as well
    te = pu.TrajectoryEvaluator(traj_poses, time_bounds=-0.1)
    for t in [0.0, len(poses6), 0.5, len(poses6) - 0.5]:
        with pytest.raises(ValueError):
            te.pose_at(t)
    assert np.allclose(te.pose_at(0.75), p_075)

    # some elements of ts vec is out of bounds
    with pytest.raises(ValueError):
        te.poses_at(ts)
    assert np.allclose(te.poses_at(np.array([0.75])), np.array([p_075]))


def test_no_scipy_exp_log_ops(poses6: List[pu.Pose6]):
    """Test no scipy version of exp/log functions."""
    rot_vecs = np.zeros((len(poses6), 3))
    rot_mats = np.zeros((len(poses6), 3, 3))
    for i, p in enumerate(poses6):
        rv = p[:3]
        rot_vecs[i] = rv
        rot_mats[i] = pu._no_scipy_exp_rot_vec(rv)
        assert np.allclose(rot_mats[i], pu.exp_rot_vec(rv))
        assert np.allclose(pu.exp_pose6(p), pu._no_scipy_exp_pose6(p))

    rot_mats = pu._no_scipy_exp_rot_vec(rot_vecs)

    assert np.allclose(pu._no_scipy_exp_rot_vec(rot_vecs), pu.exp_rot_vec(rot_vecs))
    assert np.allclose(pu.exp_pose6(np.array(poses6)), pu._no_scipy_exp_pose6(np.array(poses6)))

    for i in range(len(rot_mats)):
        assert np.allclose(pu._no_scipy_log_rot_mat(rot_mats[i]), rot_vecs[i])

    assert np.allclose(pu._no_scipy_log_rot_mat(rot_mats), pu.log_rot_mat(rot_mats))
