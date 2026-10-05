import numpy as np
import pytest

from ouster.sdk import core
from ouster.sdk.util.pose_util import TrajectoryEvaluator


def translation_trajectory(start, end):
    first = np.eye(4)
    last = np.eye(4)
    last[0, 3] = 10
    return [(start, first), (end, last)]


def evaluate(evaluator, timestamps, batch):
    if batch:
        return evaluator.poses_at(timestamps)
    return np.array([evaluator.pose_at(ts) for ts in timestamps])


@pytest.mark.parametrize("knot_type", [int, np.int64, np.uint64])
@pytest.mark.parametrize("query_type", [int, np.int64, np.uint64])
@pytest.mark.parametrize("batch", [False, True])
def test_integer_timestamps_within_bounds(knot_type, query_type, batch):
    evaluator = TrajectoryEvaluator(translation_trajectory(knot_type(100), knot_type(200)))
    queries = [query_type(value) for value in [100, 125, 150, 150, 175, 200]]
    poses = evaluate(evaluator, queries, batch)
    np.testing.assert_allclose(poses[:, 0, 3], [0, 2.5, 5, 5, 7.5, 10])
    np.testing.assert_allclose(poses[:, :3, :3], np.tile(np.eye(3), (6, 1, 1)))


@pytest.mark.parametrize("timestamp_type", [int, np.int64, np.uint64])
@pytest.mark.parametrize("time_bounds", [None, 0.5])
@pytest.mark.parametrize("batch", [False, True])
def test_integer_timestamps_extrapolate(timestamp_type, time_bounds, batch):
    evaluator = TrajectoryEvaluator(
        translation_trajectory(timestamp_type(100), timestamp_type(200)),
        time_bounds=time_bounds,
    )
    queries = [timestamp_type(value) for value in [50, 100, 200, 250]]
    poses = evaluate(evaluator, queries, batch)
    np.testing.assert_allclose(poses[:, 0, 3], [-5, 0, 10, 15])


@pytest.mark.parametrize("start", [1_700_000_000_000_000_000, 2**63 + 1234, 2**64 - 101])
@pytest.mark.parametrize("timestamp_type", [int, np.uint64])
@pytest.mark.parametrize("batch", [False, True])
def test_nanosecond_precision(start, timestamp_type, batch):
    evaluator = TrajectoryEvaluator(
        translation_trajectory(timestamp_type(start), timestamp_type(start + 20))
    )
    queries = [timestamp_type(start + offset) for offset in [0, 1, 10, 19, 20]]
    poses = evaluate(evaluator, queries, batch)
    np.testing.assert_allclose(poses[:, 0, 3], [0, 0.5, 5, 9.5, 10])


@pytest.mark.parametrize("query", [49, 251])
@pytest.mark.parametrize("batch", [False, True])
def test_unsigned_timestamps_outside_bounds(query, batch):
    evaluator = TrajectoryEvaluator(
        translation_trajectory(np.uint64(100), np.uint64(200)), time_bounds=0.5
    )
    with pytest.raises(ValueError, match="timestamp range"):
        evaluate(evaluator, [np.uint64(query)], batch)


@pytest.mark.parametrize("knot_type", [int, np.uint64])
@pytest.mark.parametrize("remap", [False, True])
def test_unsigned_frame_timestamps_and_dewarp(knot_type, remap):
    start = 1_700_000_000_000_000_000
    evaluator = TrajectoryEvaluator(translation_trajectory(knot_type(start), knot_type(start + 20)))
    frame = core.LidarFrame(2, 4, [], 4)
    frame.status[:] = [1, 0, 1, 1]
    times = np.array([start + 4, 0, start + 10, start + 16], dtype=np.uint64)
    frame.timestamp[:] = 1 if remap else times
    frame.body_to_world[:] = np.eye(4)
    frame.body_to_world[1, 0, 3] = 123
    assert evaluator(frame, col_ts=times if remap else None) is frame
    np.testing.assert_allclose(frame.body_to_world[:, 0, 3], [2, 123, 5, 8])
    points = np.zeros((2, 4, 3), dtype=np.float64)
    transformed = core.dewarp(points, frame.body_to_world)
    np.testing.assert_allclose(transformed[:, :, 0], [[2, 123, 5, 8]] * 2)


@pytest.mark.parametrize("timestamp_type", [np.int64, np.uint64])
def test_trajectory_retains_original_timestamp_values(timestamp_type):
    original = translation_trajectory(timestamp_type(100), timestamp_type(200))
    evaluator = TrajectoryEvaluator(original)
    assert isinstance(evaluator[0][0], timestamp_type)
    assert evaluator[0][0] == original[0][0]
    queries = np.array([100, 150, 200], dtype=timestamp_type)
    evaluator.poses_at(queries)
    np.testing.assert_array_equal(queries, [100, 150, 200])
