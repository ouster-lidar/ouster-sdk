from contextlib import closing
import copy

import numpy as np
import pytest

from ouster.sdk import core, open_source
from ouster.sdk.algorithm import (
    align_clouds,
    normals,
    point_to_plane_align,
    point_to_point_align,
)
from ouster.sdk.core import get_rot_matrix_to_align_to_gravity

IMU_STATUS_VALID_MASK = 0x01
NORMAL_EPS = 1e-6

POSE_DELTA_CASES = [
    pytest.param(
        "pose_delta_1_128.osf",
        np.array(
            [
                [0.98251097, -0.18554714, 0.01563489, 2.17698953],
                [0.18562688, 0.98261296, -0.00380030, 0.46332984],
                [-0.01465791, 0.00663609, 0.99987055, 0.29383690],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        np.array(
            [
                [0.98251097, -0.18554714, 0.01563489, 2.17698953],
                [0.18562688, 0.98261296, -0.00380030, 0.46332984],
                [-0.01465791, 0.00663609, 0.99987055, 0.29383690],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        id="pose_delta_1",
    ),
    pytest.param(
        "pose_delta_2_128.osf",
        np.array(
            [
                [0.99997934, -0.00546031, -0.00339061, 0.67199152],
                [0.00546738, 0.99998289, 0.00208084, 0.01065750],
                [0.00337919, -0.00209933, 0.99999209, 0.10954901],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        np.array(
            [
                [0.99997934, -0.00546031, -0.00339061, 0.67199152],
                [0.00546738, 0.99998289, 0.00208084, 0.01065750],
                [0.00337919, -0.00209933, 0.99999209, 0.10954901],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        id="pose_delta_2",
    ),
]

# Frame-based align_clouds does internal feature extraction + coarse
# alignment + ICP in C++.
MAX_FRAME_TRANSLATION_ERROR_M = 1.00
MAX_FRAME_ROTATION_ERROR_RAD = np.deg2rad(8.0)

# Points+normals use C++ ICP which can converge differently across platforms.
MAX_TRANSLATION_ERROR_M = 2.20
MAX_ROTATION_ERROR_RAD = np.deg2rad(8.0)

# Points-only can differ more because it does not use normal constraints.
MAX_POINTS_TRANSLATION_ERROR_M = 2.20
MAX_POINTS_ROTATION_ERROR_RAD = np.deg2rad(8.0)


def _first_frame_pair(osf_path, sensor_a=0, sensor_b=1):
    # Take the first frame seen from each sensor, even when collation yields
    # them in separate frame sets (legacy OSFs with out-of-order data never
    # produce a set containing both sensors).
    frames = {}
    with closing(open_source(str(osf_path), collate=True)) as src:
        for frame_sets in src:
            if frame_sets is None:
                continue
            if len(frame_sets) <= max(sensor_a, sensor_b):
                continue
            for idx in (sensor_a, sensor_b):
                if idx not in frames and frame_sets[idx] is not None:
                    frames[idx] = frame_sets[idx]
            if sensor_a in frames and sensor_b in frames:
                return frames[sensor_a], frames[sensor_b]
    raise RuntimeError(
        f"Could not find frames for both sensor {sensor_a} and sensor {sensor_b}"
    )


def compute_plumb_rotation_from_frame_imu(frame, fix_yaw=False):
    if (not frame.has_field(core.ChanField.IMU_STATUS) or
            not frame.has_field(core.ChanField.IMU_ACC)):
        return None

    imu_status = np.asarray(frame.field(core.ChanField.IMU_STATUS)).reshape(-1)
    imu_acc = np.asarray(frame.field(core.ChanField.IMU_ACC), dtype=np.float64)
    if imu_acc.ndim != 2 or imu_acc.shape[1] != 3:
        return None
    if imu_status.size == 0 or imu_acc.shape[0] != imu_status.size:
        return None

    valid = ((imu_status & IMU_STATUS_VALID_MASK) != 0) & np.all(
        np.isfinite(imu_acc), axis=1
    )
    if not np.any(valid):
        return None

    chosen_acc = None
    if frame.has_field(core.ChanField.IMU_TIMESTAMP):
        imu_ts = np.asarray(frame.field(core.ChanField.IMU_TIMESTAMP)).reshape(-1)
        if imu_ts.size == imu_status.size:
            ref_ts = int(frame.timestamp[frame.get_first_valid_column()])
            valid_idx = np.flatnonzero(valid)
            if valid_idx.size > 0:
                dt = np.abs(imu_ts[valid_idx].astype(np.int64) - ref_ts)
                k = min(5, int(valid_idx.size))
                nearest = valid_idx[np.argpartition(dt, k - 1)[:k]]
                chosen_acc = imu_acc[nearest].mean(axis=0)

    if chosen_acc is None:
        chosen_acc = imu_acc[valid].mean(axis=0)

    imu_to_points_rotation = np.eye(3, dtype=np.float64)
    if frame.sensor_info is not None:
        imu_to_sensor = np.asarray(
            frame.sensor_info.imu_to_sensor_transform[:3, :3], dtype=np.float64
        )
        sensor_to_points = np.asarray(frame.sensor_info.sensor_to_body[:3, :3], dtype=np.float64)
        imu_to_points_rotation = sensor_to_points @ imu_to_sensor

    chosen_acc = imu_to_points_rotation @ chosen_acc
    norm = float(np.linalg.norm(chosen_acc))
    if not np.isfinite(norm) or norm <= 1e-6:
        return None

    rot = np.asarray(
        get_rot_matrix_to_align_to_gravity(
            float(chosen_acc[0]),
            float(chosen_acc[1]),
            float(chosen_acc[2]),
            fix_yaw=fix_yaw,
        ),
        dtype=np.float64,
    )
    if rot.shape != (3, 3) or not np.all(np.isfinite(rot)):
        return None
    return rot


def prepare_frame_points_like_cpp(frame, plumb_rotation):
    """Mirror C++ prepare_frame_features() for point-cloud API parity."""
    if frame.sensor_info is None:
        raise RuntimeError("frame.sensor_info is missing; cannot compute XYZ.")

    range_img = np.asarray(frame.field(core.ChanField.RANGE), dtype=np.uint32)
    h, w = range_img.shape
    try:
        first_col = int(frame.get_first_valid_column())
        last_col = int(frame.get_last_valid_column())
    except RuntimeError:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.float64)
    if last_col < first_col:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.float64)

    info_for_xyz = copy.deepcopy(frame.sensor_info)
    if plumb_rotation is not None:
        ext = np.asarray(info_for_xyz.sensor_to_body, dtype=np.float64).copy()
        ext[:3, :3] = plumb_rotation @ ext[:3, :3]
        info_for_xyz.sensor_to_body = ext
    xyz_flat = np.asarray(core.XYZLut(info_for_xyz, use_extrinsics=True)(frame),
                          dtype=np.float64)

    poses = np.asarray(frame.body_to_world, dtype=np.float64)
    first_pose_inv = np.linalg.inv(poses[first_col])
    rel_poses = np.einsum("ij,wjk->wik", first_pose_inv, poses)
    dewarped_flat = np.asarray(core.dewarp(xyz_flat, np.ascontiguousarray(rel_poses)),
                               dtype=np.float64)
    dewarped = np.ascontiguousarray(dewarped_flat.reshape(h, w, 3))

    normals_img = None
    if frame.has_field(core.ChanField.NORMALS):
        raw_normals = np.asarray(frame.field(core.ChanField.NORMALS), dtype=np.float64)
        if raw_normals.ndim == 3 and raw_normals.shape == (h, w, 3):
            normals_img = raw_normals
        elif raw_normals.ndim == 2 and raw_normals.shape == (h * w, 3):
            normals_img = raw_normals.reshape(h, w, 3)
        if normals_img is not None and plumb_rotation is not None:
            normals_img = np.einsum("ij,hwj->hwi", plumb_rotation, normals_img)
    else:
        sensor_origins = np.asarray(rel_poses[:, :3, 3], dtype=np.float64)
        if plumb_rotation is not None:
            sensor_origins = sensor_origins @ plumb_rotation.T
        try:
            computed = np.asarray(
                normals(
                    np.ascontiguousarray(dewarped),
                    np.ascontiguousarray(range_img),
                    np.ascontiguousarray(sensor_origins),
                ),
                dtype=np.float64,
            )
            if computed.ndim == 3 and computed.shape == (h, w, 3):
                normals_img = computed
            elif computed.ndim == 2 and computed.shape == (h * w, 3):
                normals_img = computed.reshape(h, w, 3)
        except Exception:
            normals_img = None

    status = np.asarray(frame.status, dtype=np.uint32).reshape(-1)
    max_valid = h * (last_col - first_col + 1)

    if normals_img is None:
        valid_points = np.empty((max_valid, 3), dtype=np.float64)
        idx_out = 0
        for col in range(first_col, last_col + 1):
            if status[col] == 0:
                continue
            for row in range(h):
                if range_img[row, col] == 0:
                    continue
                p = np.asarray(dewarped[row, col], dtype=np.float64)
                if not np.all(np.isfinite(p)):
                    continue
                valid_points[idx_out] = p
                idx_out += 1
        return np.ascontiguousarray(valid_points[:idx_out]), np.empty((0, 3), dtype=np.float64)

    valid_points = np.empty((max_valid, 3), dtype=np.float64)
    valid_normals = np.empty((max_valid, 3), dtype=np.float64)
    idx_out = 0
    for col in range(first_col, last_col + 1):
        if status[col] == 0:
            continue
        for row in range(h):
            if range_img[row, col] == 0:
                continue
            p = np.asarray(dewarped[row, col], dtype=np.float64)
            if not np.all(np.isfinite(p)):
                continue
            n = np.asarray(normals_img[row, col], dtype=np.float64)
            n_norm = float(np.linalg.norm(n))
            if (not np.all(np.isfinite(n))) or (n_norm <= NORMAL_EPS):
                continue
            valid_points[idx_out] = p
            valid_normals[idx_out] = n / n_norm
            idx_out += 1

    return (
        np.ascontiguousarray(valid_points[:idx_out]),
        np.ascontiguousarray(valid_normals[:idx_out]),
    )


def _pose_error(actual: np.ndarray, expected: np.ndarray):
    delta_r = actual[:3, :3] @ expected[:3, :3].T
    trace = float(np.trace(delta_r))
    cos_theta = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
    rot_err = float(np.arccos(cos_theta))
    trans_err = float(np.linalg.norm(actual[:3, 3] - expected[:3, 3]))
    return trans_err, rot_err


def test_point_to_point_align():
    source = np.array(
        [
            [x, y, z]
            for x in (-1.0, 0.0, 1.0)
            for y in (-1.0, 0.0, 1.0)
            for z in (-1.0, 0.0, 1.0)
        ],
        dtype=np.float64,
    )
    translation = np.array([0.1, -0.05, 0.025])
    target = np.ascontiguousarray(source + translation)

    actual = point_to_point_align(source, target, max_corr_dist=0.5)

    np.testing.assert_allclose(actual[:3, :3], np.eye(3), atol=1e-10)
    np.testing.assert_allclose(actual[:3, 3], translation, atol=1e-10)


def test_point_to_plane_align():
    source = np.array(
        [[x, y, 0.0] for x in range(5) for y in range(5)],
        dtype=np.float64,
    )
    target = source.copy()
    target[:, 2] = 0.2
    normals = np.tile([0.0, 0.0, 1.0], (source.shape[0], 1))

    actual = point_to_plane_align(
        source, target, normals, normals, max_corr_dist=0.5
    )

    np.testing.assert_allclose(actual[:3, :3], np.eye(3), atol=1e-10)
    np.testing.assert_allclose(actual[:3, 3], [0.0, 0.0, 0.2], atol=1e-9)


def _apply_gravity_aligned_extrinsics(frame, plumb_rotation):
    """Pre-rotate sensor_to_body so the extrinsic is gravity-aligned.

    align_clouds(frame, frame) no longer reads IMU fields or performs any
    internal gravity estimation (see align_clouds.h); the caller must provide
    a gravity-aligned sensor_to_body before calling.
    """
    if plumb_rotation is None:
        return
    ext = np.asarray(frame.sensor_info.sensor_to_body, dtype=np.float64).copy()
    ext[:3, :3] = plumb_rotation @ ext[:3, :3]
    frame.sensor_info.sensor_to_body = ext


@pytest.mark.parametrize(
    "osf_name,expected_frame_pose,expected_points_pose",
    POSE_DELTA_CASES,
)
def test_align_clouds_osf_cases(
    test_data_dir,
    osf_name,
    expected_frame_pose,
    expected_points_pose,
):
    # Stored expectations predate the source_to_target_transform API convention.
    expected_frame_pose = np.linalg.inv(expected_frame_pose)
    expected_points_pose = np.linalg.inv(expected_points_pose)

    osf_path = test_data_dir / "osfs" / osf_name
    frame1, frame2 = _first_frame_pair(osf_path, sensor_a=0, sensor_b=1)

    rot1 = compute_plumb_rotation_from_frame_imu(frame1, fix_yaw=False)
    rot2 = compute_plumb_rotation_from_frame_imu(frame2, fix_yaw=False)
    plumb1 = rot1 if (rot1 is not None and rot2 is not None) else None
    plumb2 = rot2 if (rot1 is not None and rot2 is not None) else None

    points1, normals1 = prepare_frame_points_like_cpp(frame1, plumb1)
    points2, normals2 = prepare_frame_points_like_cpp(frame2, plumb2)

    # Frame overload under the current contract: gravity-align the extrinsics
    # first, then align. The frame path thus shares the plumbed convention
    # with the points paths.
    _apply_gravity_aligned_extrinsics(frame1, plumb1)
    _apply_gravity_aligned_extrinsics(frame2, plumb2)
    pose_frame = np.asarray(align_clouds(frame1, frame2), dtype=np.float64)

    pose_points = np.asarray(align_clouds(points1, points2), dtype=np.float64)
    pose_points_normals = np.asarray(
        align_clouds(points1, normals1, points2, normals2),
        dtype=np.float64,
    )

    frame_trans_err, frame_rot_err = _pose_error(pose_frame, expected_frame_pose)
    points_trans_err, points_rot_err = _pose_error(
        pose_points, expected_points_pose
    )
    pn_trans_err, pn_rot_err = _pose_error(
        pose_points_normals, expected_points_pose
    )

    assert frame_trans_err <= MAX_FRAME_TRANSLATION_ERROR_M
    assert frame_rot_err <= MAX_FRAME_ROTATION_ERROR_RAD
    assert points_trans_err <= MAX_POINTS_TRANSLATION_ERROR_M
    assert points_rot_err <= MAX_POINTS_ROTATION_ERROR_RAD
    assert pn_trans_err <= MAX_TRANSLATION_ERROR_M
    assert pn_rot_err <= MAX_ROTATION_ERROR_RAD
