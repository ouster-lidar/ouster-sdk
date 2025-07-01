import numpy as np
from ouster.sdk import core

# ![doc-stag-python-scan-add-field]
lidar_scan = core.LidarScan(128, 1024)
lidar_scan.add_field("my-custom-field", np.uint8, ())
lidar_scan.field("my-custom-field")[:] = 1  # set all pixels
lidar_scan.field("my-custom-field")[10:20, 10:20] = 255  # set a block of pixels
# ![doc-etag-python-scan-add-field]
