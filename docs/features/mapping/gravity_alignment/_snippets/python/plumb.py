import sys

import numpy as np

from ouster.sdk import core, open_source
from ouster.sdk.algorithm import plumb, plumb_imu, plumb_walls


def plumb_example(source_file):
    # [doc-stag-plumb]
    source = open_source(source_file)
    frames: core.FrameSet = next(iter(source))

    # IMU first, then the wall refinement seeded with it. Each stage logs
    # what it found; the result is None when neither produced a correction.
    correction = plumb(frames)
    if correction is None:
        print("could not plumb the frame set", file=sys.stderr)
        return np.eye(4)

    for info in source.sensor_info:
        ext = np.asarray(info.sensor_to_body)
        info.sensor_to_body = correction @ ext
        # drop any XYZLut cached with the old extrinsics
        info.clear_cache()
    # [doc-etag-plumb]

    return correction


def plumb_imu_example(source_file):
    # [doc-stag-plumb-imu]
    source = open_source(source_file)
    frames: core.FrameSet = next(iter(source))

    # The accelerometer alone. No wall search, so it is cheap and works
    # where there is no vertical structure, but it needs usable IMU data.
    imu_correction = plumb_imu(frames)
    if imu_correction is None:
        print("no usable IMU data to plumb from", file=sys.stderr)
        return imu_correction

    for info in source.sensor_info:
        ext = np.asarray(info.sensor_to_body)
        info.sensor_to_body = imu_correction @ ext
        # drop any XYZLut cached with the old extrinsics
        info.clear_cache()
    # [doc-etag-plumb-imu]

    return imu_correction


def plumb_walls_example(source_file):
    # [doc-stag-plumb-walls]
    source = open_source(source_file)
    frames: core.FrameSet = next(iter(source))

    # Identity means "refine the orientation the extrinsics already
    # carry". Pass an IMU-derived rotation to refine that instead.
    walls_correction = plumb_walls(frames, np.eye(4))
    if walls_correction is None:
        print("wall refinement did not converge", file=sys.stderr)
        return walls_correction

    for info in source.sensor_info:
        ext = np.asarray(info.sensor_to_body)
        info.sensor_to_body = walls_correction @ ext
        # drop any XYZLut cached with the old extrinsics
        info.clear_cache()
    # [doc-etag-plumb-walls]

    return walls_correction
