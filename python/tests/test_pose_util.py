from typing import List
import numpy as np
import math
import time
import pytest
import ouster.sdk.util.pose_util as pu
from ouster.sdk.core import dewarp, transform
import ouster.sdk.core as core


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
          f"({t_pose_interp / t_pose_te_one_by_one:.02f}x)")
    vec_speedup = ""
    if t_pose_te_vec > 0:
        vec_speedup = f"({t_pose_te_one_by_one / t_pose_te_vec:.02f}x)"
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


def test_rotation_alignment_vector1():
    accel_x, accel_y, accel_z = 1, 0, 1
    rotation_matrix = pu.get_rot_matrix_to_align_to_gravity(accel_x, accel_y, accel_z)
    aligned_vector = rotation_matrix @ pu.normalize_vector(np.array([accel_x, accel_y, accel_z]))
    # align with gravity
    expected_vector = np.array([0, 0, 1])
    np.testing.assert_almost_equal(aligned_vector, expected_vector, decimal=6)


def test_rotation_alignment_vector2():
    accel_x, accel_y, accel_z = 0, 1, 1
    rotation_matrix = pu.get_rot_matrix_to_align_to_gravity(accel_x, accel_y, accel_z)
    aligned_vector = rotation_matrix @ pu.normalize_vector(np.array([accel_x, accel_y, accel_z]))
    # align with gravity
    expected_vector = np.array([0, 0, 1])
    np.testing.assert_almost_equal(aligned_vector, expected_vector, decimal=6)


def test_transform_N_3():
    # Define known input points of shape (10, 3)
    points_c = np.array([[1.0, 2.0, 3.0],
                       [4.0, 5.0, 6.0],
                       [7.0, 8.0, 9.0],
                       [10.0, 11.0, 12.0],
                       [13.0, 14.0, 15.0],
                       [16.0, 17.0, 18.0],
                       [19.0, 20.0, 21.0],
                       [22.0, 23.0, 24.0],
                       [25.0, 26.0, 27.0],
                       [28.0, 29.0, 30.0]], order='C')

    # Define a non-identity transformation matrix (4x4)
    # yawl 30 degree with (1, 2 ,-1) translation
    transformation_matrix_c = np.array([[0.866, -0.5, 0.0, 1.0],
                                      [0.5, 0.866, 0.0, 2.0],
                                      [0.0, 0.0, 1.0, -1.0],
                                      [0.0, 0.0, 0.0, 1.0]], order='C')

    # Convert to F-order
    points_f = np.asfortranarray(points_c)
    transformation_matrix_f = np.asfortranarray(transformation_matrix_c)

    # Expected transformed points (manually calculated)
    expected_points = np.array([[0.866, 4.232, 2],
                                [1.964, 8.33, 5],
                                [3.062, 12.428, 8],
                                [4.16, 16.526, 11],
                                [5.258, 20.624, 14],
                                [6.356, 24.722, 17],
                                [7.454, 28.82, 20],
                                [8.552, 32.918, 23],
                                [9.65, 37.016, 26],
                                [10.748, 41.114, 29]])

    transformed_points_cc = transform(points_c, transformation_matrix_c)
    transformed_points_cf = transform(points_c, transformation_matrix_f)
    transformed_points_fc = transform(points_f, transformation_matrix_c)
    transformed_points_ff = transform(points_f, transformation_matrix_f)
    np.testing.assert_almost_equal(transformed_points_cc, expected_points, decimal=5)
    np.testing.assert_almost_equal(transformed_points_cf, expected_points, decimal=5)
    np.testing.assert_almost_equal(transformed_points_fc, expected_points, decimal=5)
    np.testing.assert_almost_equal(transformed_points_ff, expected_points, decimal=5)


def test_transform_N_M_3():
    # Define known input points of shape (2, 4, 3)
    points_c = np.array([[[1.0, 2.0, 3.0],
                        [4.0, 5.0, 6.0],
                        [7.0, 8.0, 9.0],
                        [10.0, 11.0, 12.0]],

                       [[13.0, 14.0, 15.0],
                        [16.0, 17.0, 18.0],
                        [19.0, 20.0, 21.0],
                        [22.0, 23.0, 24.0]]], order='C')

    # Define a non-identity transformation matrix (4x4)
    # yawl 30 degree with (1, 2 ,-1) translation
    transformation_matrix_c = np.array([[0.866, -0.5, 0.0, 1.0],
                                      [0.5, 0.866, 0.0, 2.0],
                                      [0.0, 0.0, 1.0, -1.0],
                                      [0.0, 0.0, 0.0, 1.0]], order='C')

    # Convert to F-order
    points_f = np.asfortranarray(points_c)
    assert (points_f == points_c).all()
    transformation_matrix_f = np.asfortranarray(transformation_matrix_c)
    assert (transformation_matrix_f == transformation_matrix_c).all()

    # Expected transformed points (manually calculated)

    expected_points = np.array([[[0.866, 4.232, 2],
                                 [1.964, 8.33, 5],
                                 [3.062, 12.428, 8],
                                 [4.16, 16.526, 11]],

                                [[5.258, 20.624, 14],
                                 [6.356, 24.722, 17],
                                 [7.454, 28.82, 20],
                                 [8.552, 32.918, 23]]])

    transformed_points_cc = transform(points_c, transformation_matrix_c)
    transformed_points_cf = transform(points_c, transformation_matrix_f)
    transformed_points_fc = transform(points_f, transformation_matrix_c)
    transformed_points_ff = transform(points_f, transformation_matrix_f)
    np.testing.assert_almost_equal(transformed_points_cc, expected_points, decimal=5)
    np.testing.assert_almost_equal(transformed_points_cf, expected_points, decimal=5)
    np.testing.assert_almost_equal(transformed_points_fc, expected_points, decimal=5)
    np.testing.assert_almost_equal(transformed_points_ff, expected_points, decimal=5)


def test_dewarp():
    poses = np.array([[1, 0, 0, 1, 0, 1, 0, -2, 0, 0, 1, 3, 0, 0, 0, 1] for _ in range(4)])
    points = np.array([[i - 3, i + 1, i + 2] for i in range(2 * 4)])
    num_poses = poses.shape[0]
    pts_per_pose = int(points.shape[0] / poses.shape[0])

    poses_reshaped = poses.reshape(num_poses, 4, 4)
    points_reshaped = points.reshape(pts_per_pose, num_poses, 3)

    poses_c = np.ascontiguousarray(poses_reshaped)
    points_c = np.ascontiguousarray(points_reshaped)
    poses_f = np.asfortranarray(poses_reshaped)
    points_f = np.asfortranarray(points_reshaped)

    expected_points = np.array([[[-2, -1, 5],
                                 [-1, 0, 6],
                                 [0, 1, 7],
                                 [1, 2, 8]],

                                [[2, 3, 9],
                                 [3, 4, 10],
                                 [4, 5, 11],
                                 [5, 6, 12]]])

    dewarped_points_cc = dewarp(points_c, poses_c)
    dewarped_points_cf = dewarp(points_c, poses_f)
    dewarped_points_fc = dewarp(points_f, poses_c)
    dewarped_points_ff = dewarp(points_f, poses_f)

    np.testing.assert_allclose(dewarped_points_cc.shape, (2, 4, 3))
    np.testing.assert_allclose(dewarped_points_cc, expected_points, rtol=1e-5, atol=1e-8)
    np.testing.assert_allclose(dewarped_points_cf.shape, (2, 4, 3))
    np.testing.assert_allclose(dewarped_points_cf, expected_points, rtol=1e-5, atol=1e-8)
    np.testing.assert_allclose(dewarped_points_fc.shape, (2, 4, 3))
    np.testing.assert_allclose(dewarped_points_fc, expected_points, rtol=1e-5, atol=1e-8)
    np.testing.assert_allclose(dewarped_points_ff.shape, (2, 4, 3))
    np.testing.assert_allclose(dewarped_points_ff, expected_points, rtol=1e-5, atol=1e-8)


def test_euler_pose_to_matrix():
    # Input pose: [roll, pitch, yaw, x, y, z]
    euler_pose = np.array([0.1, 0.2, 0.3, 1, 2, 3], dtype=np.float64)
    transform_euler = core.euler_pose_to_matrix(euler_pose)

    expected = np.array([
        [0.93629336, -0.27509585, 0.21835066, 1],
        [0.28962948, 0.95642509, -0.03695701, 2],
        [-0.19866933, 0.0978434, 0.97517033, 3],
        [0, 0, 0, 1]])

    assert np.allclose(transform_euler, expected, atol=1e-4)


def test_quaternion_pose_to_matrix():
    # Input pose: [qw, qx, qy, qz, x, y, z]
    quaternion_pose = np.array([0.8446, 0.1913, 0.4619, 0.1913, 1, 2, 3], dtype=np.float64)
    transform_quaternion = core.quaternion_pose_to_matrix(quaternion_pose)

    expected = np.array([
        [0.50005148, -0.14643681, 0.85352491, 1],
        [0.49992082, 0.85360145, -0.14643681, 2],
        [-0.70712636, 0.49992082, 0.50005148, 3],
        [0, 0, 0, 1]])

    assert np.allclose(transform_quaternion, expected, atol=1e-4)


def test_interp_pose():
    prev_ts = 101000
    curr_ts = 102000
    next_ts = 103000

    prev_pose = np.identity(4)
    curr_pose = np.array([
        [0.950564, -0.29552, -0.0953745, 5],
        [0.294044, 0.955336, -0.0295028, 3],
        [0.0998334, 0, 0.995004, 2],
        [0.0, 0.0, 0.0, 1.0]
    ])
    next_pose = np.array([
        [0.879923, -0.389418, -0.272192, 0],
        [0.372026, 0.921061, -0.115081, 0],
        [0.29552, 0, 0.955336, 0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    tss = np.array([prev_ts, curr_ts, next_ts])
    poses = np.array([prev_pose, curr_pose, next_pose])

    # Inquiring timestamps
    inquiring_ts = np.array([
            100000, 100500, 101000, 101500,
            102000, 102500, 103000, 103500
    ])

    # Expected poses based on the provided output
    expected_poses = np.array([
        np.array([
            [9.50563566e-01, 2.94044472e-01, 9.98336350e-02, -5.83461852e+00],
            [-2.95520849e-01, 9.55336290e-01, -2.79214613e-08, -1.38840457e+00],
            [-9.53747027e-02, -2.95028941e-02, 9.95004143e-01, -1.42462609e+00],
            [0.0, 0.0, 0.0, 1.0]
        ]),
        np.array([
            [0.98756338, 0.149066, 0.04997893, -2.84746798],
            [-0.14943741, 0.98876405, 0.00375782, -0.91059756],
            [-0.0488572, -0.01117981, 0.9987432, -0.7874577],
            [0.0, 0.0, 0.0, 1.0]
        ]),
        np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]),
        np.array([
            [0.98756338, -0.14943741, -0.0488572, 2.63750479],
            [0.149066, 0.98876405, -0.01117981, 1.31602317],
            [0.04997893, 0.00375782, 0.9987432, 0.93220328],
            [0.0, 0.0, 0.0, 1.0]
        ]),
        np.array([
            [9.50563566e-01, -2.95520849e-01, -9.53747027e-02, 5.00000000e+00],
            [2.94044472e-01, 9.55336290e-01, -2.95028941e-02, 3.00000000e+00],
            [9.98336350e-02, -2.79214613e-08, 9.95004143e-01, 2.00000000e+00],
            [0.0, 0.0, 0.0, 1.0]
        ]),
        np.array([
            [0.9208184, -0.34289665, -0.18578365, 2.41554879],
            [0.33559594, 0.93936924, -0.07042739, 1.5451861],
            [0.1986689, 0.00250261, 0.98006331, 1.14334982],
            [0.0, 0.0, 0.0, 1.0]
        ]),
        np.array([
            [8.79923149e-01, -3.89418384e-01, -2.72192394e-01, -2.23569899e-07],
            [3.72025983e-01, 9.21060403e-01, -1.15081184e-01, -1.19328240e-06],
            [2.95520591e-01, -1.60280748e-08, 9.55336219e-01, 3.73615875e-06],
            [0.0, 0.0, 0.0, 1.0]
        ]),
        np.array([
            [0.82838856, -0.43450466, -0.35352245, -2.21632003],
            [0.40287952, 0.90063797, -0.16290697, -1.61678417],
            [0.38917989, -0.00747664, 0.9211313, -1.41550338],
            [0.0, 0.0, 0.0, 1.0]
        ])
    ])

    interp_poses = core.interp_pose(inquiring_ts, tss, poses)

    assert len(interp_poses) == len(expected_poses), (
        f"Pose count mismatch: expected {len(expected_poses)}, got {len(interp_poses)}"
    )

    for idx, (computed_pose, expected_pose) in enumerate(zip(interp_poses, expected_poses)):
        assert np.allclose(computed_pose, expected_pose, atol=1e-4), (
            f"Mismatch at pose index {idx}:\nComputed:\n{computed_pose}\nExpected:\n{expected_pose}"
        )
