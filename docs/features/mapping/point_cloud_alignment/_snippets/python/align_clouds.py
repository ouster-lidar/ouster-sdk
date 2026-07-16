from ouster.sdk import core, open_source
from ouster.sdk.algorithm import align_clouds


def align_pairwise_example(source_file):
    # [doc-stag-alignment-pairwise]
    source = open_source(source_file)
    frames: core.FrameSet = next(iter(source))
    target_frame = frames[0]
    source_frame = frames[1]
    source_to_target_transform = align_clouds(source_frame, target_frame)
    aligned_source_extrinsic = source_to_target_transform @ (
        source_frame.sensor_info.sensor_to_body)
    # [doc-etag-alignment-pairwise]

    return aligned_source_extrinsic


def align_frame_set_example(source_file):
    # [doc-stag-alignment-frame-set]
    source = open_source(source_file)
    frames: core.FrameSet = next(iter(source))
    aligned_extrinsics = align_clouds(frames)

    for i, extrinsic in enumerate(aligned_extrinsics):
        source.sensor_info[i].sensor_to_body = extrinsic

    # [doc-etag-alignment-frame-set]

    return aligned_extrinsics
