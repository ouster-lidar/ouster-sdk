from __future__ import annotations

from typing import Any

# [doc-stag-localization-imports]
# Other imports
import math
from ouster.sdk import open_source
from ouster.sdk import mapping
from ouster.sdk import core
# [doc-etag-localization-imports]


def run_localization_once(source_file: str, map_file: str) -> None:
    """Open a source and run localization against a pre-built map."""

    # [doc-stag-localization-config]
    config = mapping.LocalizationConfig.create("lio")
    config.min_range = 0.5
    config.max_range = 100.0
    config.voxel_size = 1.0
    config.deskew_method = "auto"
    # [doc-etag-localization-config]

    # [doc-stag-localization-loop]
    # [doc-stag-localization-engine]
    source = open_source(source_file)
    engine = mapping.LocalizationEngine.create(
        source.sensor_info,
        map_file,
        config)
    # [doc-etag-localization-engine]
    # [doc-stag-localization-loop-update]
    for frame_set in source:
        frame_set = engine.update(frame_set)
        # [doc-etag-localization-loop-update]
        # [doc-stag-localization-loop-printpose]
        for frame in frame_set.valid_frames():
            col = frame.get_last_valid_column()
            frame_pose = frame.body_to_world[col]
            frame_ts = frame.timestamp[col]
            t = frame_pose[:3, 3]
            rot = frame_pose[:3, :3]
            angles = core.matrix_to_euler(rot)  # [roll, pitch, yaw] in radians
            rad2deg = 180.0 / math.pi
            roll  = angles[0] * rad2deg
            pitch = angles[1] * rad2deg
            yaw   = angles[2] * rad2deg

            print(
                f"idx = {frame.frame_id}; ts = {frame_ts}; "
                f"XYZ: {t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f} "
                f"(R: {roll:.1f}, P: {pitch:.1f}, Y: {yaw:.1f})"
            )

        # [doc-etag-localization-loop-printpose]
    
    # [doc-etag-localization-loop]


def gnss_initial_pose(latitude: float, longitude: float) -> Any:
    """Compute an ``initial_pose`` from a GPS fix and the map-origin UTM coordinate.

    Args:
        latitude: Current sensor latitude in decimal degrees.
        longitude: Current sensor longitude in decimal degrees.

    Returns:
        A (4, 4) SE(3) identity matrix with X/Y translation set to the
        offset between the current position and the map origin.
    """
    # [doc-stag-localization-gnss-init]
    import numpy as np
    # gps_to_utm() is a placeholder — use pyproj, utm, or a similar library.
    utm_x, utm_y = gps_to_utm(latitude, longitude)  # type: ignore[name-defined]

    # The UTM coordinate where the SLAM map's origin (0, 0, 0) was recorded.
    # You must know (or log) this value when building the map.
    map_origin_utm = (500230.0, 4182030.0)

    # Build a 4×4 identity matrix and set the X/Y translation to the
    # difference between the current position and the map origin.
    config = mapping.LocalizationConfig.create("lio")
    config.initial_pose = np.eye(4)
    config.initial_pose[0, 3] = utm_x - map_origin_utm[0]   # delta-X in metres (east)
    config.initial_pose[1, 3] = utm_y - map_origin_utm[1]   # delta-Y in metres (north)
    # [doc-etag-localization-gnss-init]

    return config.initial_pose


def configure_filtering() -> "mapping.LocalizationConfig":
    """Return a config that filters out close-range and far-range points."""
    # [doc-stag-localization-dynamic-filter]
    config = mapping.LocalizationConfig.create("lio")
    # Exclude returns closer than 1 m (e.g. the robot's own chassis).
    config.min_range = 1.0
    # Exclude returns beyond 80 m to avoid noisy long-range matches.
    config.max_range = 80.0
    # [doc-etag-localization-dynamic-filter]
    return config


__all__ = [
    "run_localization_once",
    "gnss_initial_pose",
    "configure_filtering",
]
