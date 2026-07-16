from collections.abc import Sequence

import numpy as np

from ouster.sdk import core, open_source
from ouster.sdk.algorithm import GroundSegEngine


def segment_ground(
    source_file: str,
    plumb_extrinsics: Sequence[np.ndarray],
) -> np.ndarray:
    # [doc-stag-ground-seg-api]
    source = open_source(source_file)
    for info, extrinsic in zip(source.sensor_info, plumb_extrinsics):
        info.sensor_to_body = extrinsic

    frame_set = next(iter(source))
    engine = GroundSegEngine.create()

    engine.update(frame_set)
    ground_mask = frame_set[0].field(  # type: ignore[union-attr]
        core.ChanField.GROUND)
    # [doc-etag-ground-seg-api]

    return ground_mask
