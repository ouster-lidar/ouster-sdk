import sys
from tqdm import tqdm  # for progress bar
# [doc-stag-slam-lsviz]
from ouster.sdk import open_source
from ouster.sdk.viz import LidarFrameViz
from ouster.sdk.viz.accumulators_config import LidarFrameVizAccumulatorsConfig
from ouster.sdk import mapping

source_uri = sys.argv[1]
source = open_source(source_uri)
config = mapping.SlamConfig.create("lio")
slam = mapping.SlamEngine.create(source.sensor_info, config)

num_frames_to_map = 200
frames_w_poses = [
    slam.update(frame) for _, frame in
    zip(tqdm(range(num_frames_to_map), desc="Computing map"), source)
]
viz = LidarFrameViz(
    source.sensor_info,
    accumulators_config = LidarFrameVizAccumulatorsConfig(
        accum_max_num=100,
        accum_min_dist_num=0,
        accum_min_dist_meters=4
    )
)

for frame in frames_w_poses:
    viz.update(frame)

viz.draw(update=True)
viz.run()
# [doc-etag-slam-lsviz]
