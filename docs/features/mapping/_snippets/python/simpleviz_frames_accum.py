import sys

# [doc-stag-slam-simpleviz]
from ouster.sdk import open_source
from ouster.sdk.viz import SimpleViz
from ouster.sdk import mapping

source_uri = sys.argv[1]
source = open_source(source_uri)
config = mapping.SlamConfig.create("lio")
slam = mapping.SlamEngine.create(source.sensor_info, config)

def frames_w_poses():
    for frame in source:
        yield slam.update(frame)

viz = SimpleViz(
    source.sensor_info,
    accum_max_num=100,
    accum_min_dist_num=0,
    accum_min_dist_meters=4,
    rate=1,
    on_eof='stop'
)

viz.run(frames_w_poses())
# [doc-etag-slam-simpleviz]
