# [doc-stag-destagger-viz]
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from more_itertools import nth

# [doc-stag-core-frames]
from ouster.sdk import core, pcap

if len(sys.argv) < 2:
    raise SystemExit("Usage: python viz.py <pcap_path>")

pcap_path = str(Path(sys.argv[1]).expanduser())
# open a FrameSetSource directly from the PCAP/metadata pair
source = pcap.PcapFrameSetSource(pcap_path)
metadata = source.sensor_info[0]
# pull the 84th frame set (adjust the index for your dataset)
frame_set = nth(source, 84)
if frame_set is None:
    raise RuntimeError("Expected at least 85 frame sets in source")
# single-sensor PCAP; take the first LidarFrame
frame = frame_set[0]
if frame is None:
    raise RuntimeError("Expected a LidarFrame at sensor index 0")
# reflectivity in the default staggered layout
reflectivity = frame.field(core.ChanField.REFLECTIVITY).astype(np.uint32)
# destagger using the metadata intrinsics
reflectivity_destaggered = core.destagger(metadata, reflectivity)
# [doc-etag-core-frames]
plt.imshow(reflectivity_destaggered, cmap='gray', resample=False)
plt.show()
# [doc-etag-destagger-viz]
