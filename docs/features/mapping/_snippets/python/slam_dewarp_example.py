"""Minimal example of running SLAM once and dewarping the XYZ cloud."""
from contextlib import closing

# [doc-stag-slam-dewarp-imports]
# Other imports
# Other imports
# Other imports
# Other imports

from ouster import sdk
from ouster.sdk import core
from ouster.sdk import mapping
# [doc-etag-slam-dewarp-imports]



# [doc-stag-slam-dewarp-py]
def slam_dewarp_once(source_file: str):
    with closing(sdk.open_source(source_file)) as source:
        slam_config = mapping.SlamConfig.create("lio")
        slam_config.deskew_method = "auto"

        slam_engine = mapping.SlamEngine.create(source.sensor_info, slam_config)
        # Process frames
        for frame_set in source:
            deskewed = slam_engine.update(frame_set)
            # Find the first valid frame index
            frame_idx = 0
            found = False
            for i, f in enumerate(deskewed):
                if f is not None:
                    frame_idx = i
                    found = True
                    break

            if not found:
                continue

            if not source.sensor_info or frame_idx >= len(source.sensor_info):
                continue

            # Compute Cartesian coordinates (XYZ)
            xyzlut = core.XYZLut(source.sensor_info[frame_idx],
                                 use_extrinsics=True)
            frame = deskewed[frame_idx]
            if frame is None:
                continue
            xyz = xyzlut(frame)
            # Dewarp the point cloud using the trajectory from SLAM
            # Note: frame.body_to_world is populated by:
            # slam_engine.update()
            dewarped = core.dewarp(xyz, frame.body_to_world)
            return dewarped

        # [doc-etag-slam-dewarp-py]
        return None
