# [doc-stag-viz-frame-show-open-source]
from ouster.sdk import open_source
from ouster.sdk import viz

source_url = ""  # path to pcap/osf or sensor hostname

# Load a LidarFrame from a given source (could be a pcap file, OSF file, or a live sensor)
frame_set_source = open_source(source_url)
source_iter = iter(frame_set_source)
frames = next(source_iter)
viz.lf_show(frames)

idx = 10 # example index
viz.lf_show(frame_set_source[idx])  # where idx is a valid index value into the source
# [doc-etag-viz-frame-show-open-source]


# [doc-stag-viz-frame-show-slice]
# Visualize the first 3 frames simultaneously
viz.lf_show([frame_set_source[0][0], frame_set_source[1][0], frame_set_source[2][0]])

# Visualize a range of frames from the frame_set_source
viz.lf_show(frame_set_source[0:10])
# [doc-etag-viz-frame-show-slice]
