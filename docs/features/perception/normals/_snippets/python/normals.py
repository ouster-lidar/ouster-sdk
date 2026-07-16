import numpy as np

from ouster.sdk.algorithm import normals


def estimate_normals(
    xyz_destaggered: np.ndarray,
    range_destaggered: np.ndarray,
) -> np.ndarray:
    # [doc-stag-normals-api]
    sensor_origins = np.zeros(
        (range_destaggered.shape[1], 3), dtype=np.float64)

    normal_image = normals(
        xyz_destaggered,
        range_destaggered,
        sensor_origins_xyz=sensor_origins)
    # [doc-etag-normals-api]

    return normal_image
