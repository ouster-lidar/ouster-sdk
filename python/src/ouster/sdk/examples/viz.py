"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Example of interactive visualizer use.

Intended to run with `python -i -m ouster.sdk.examples.viz`

"""

from ouster import client, pcap
from ouster.sdk import viz
import os
import sys
import numpy as np


def make_checker_board(square_size, reps):
    """Makes a test checker board image."""
    img_data = np.full((square_size, square_size), 0)
    img_data = np.hstack([img_data, np.logical_xor(img_data, 1)])
    img_data = np.vstack([img_data, np.logical_xor(img_data, 1)])
    img_data = np.tile(img_data, reps)
    return img_data

meta_path = os.getenv("SAMPLE_DATA_JSON_PATH", "")
pcap_path = os.getenv("SAMPLE_DATA_PCAP_PATH", "")

if not pcap_path or not meta_path:
    print("ERROR: Please add SAMPLE_DATA_PCAP_PATH and SAMPLE_DATA_JSON_PATH to" +
          " environment variables and try again")
    sys.exit()

print(f"Using:\n\tjson: {meta_path}\n\tpcap: {pcap_path}")

meta = client.SensorInfo(open(meta_path).read())
packets = pcap.Pcap(pcap_path, meta)
scans = iter(client.Scans(packets))

point_viz = viz.PointViz("Example Viz")
viz.add_default_controls(point_viz)

# ====================================================
# Ex1: Point Cloud of the first frame of the pcap file

# Creating LidarScan visualizer (3D point cloud + field images on top)
ls_viz = viz.LidarScanViz(meta, point_viz)

# adding scan to the lidar scan viz
ls_viz.scan = next(scans)

# refresh viz data
ls_viz.draw()

print("Showing first frame, close visualizer window to continue")
point_viz.run()


# ===========================================
# Ex2: Augmenting point cloud with 3D Labels 

# Adding 3D Labels
label1 = viz.Label("Label1: [1, 2, 4]", 1, 2, 4)
point_viz.add(label1)

label2 = viz.Label("Label2: [2, 1, 4]", 2, 1, 4)
label2.set_scale(2)
point_viz.add(label2)

label3 = viz.Label("Label3: [4, 2, 1]", 4, 2, 1)
label3.set_scale(3.0)
point_viz.add(label3)

print("Added 3D labels, close visualizer window to continue")
point_viz.update()
point_viz.run()

# =====================================
# Ex3: Overlay 2D Images and 2D Labels

# Adding image 1 with aspect ratio preserved
img = viz.Image()
img_data = make_checker_board(10, (2, 4))
mask_data = np.zeros((30, 30, 4))
mask_data[:15, :15] = np.array([1, 0, 0, 1])
img.set_mask(mask_data)
img.set_image(img_data)
ypos = (0, 0.5)
xlen = (ypos[1] - ypos[0]) * img_data.shape[1] / img_data.shape[0]
xpos = (0, xlen)
img.set_position(*xpos, *ypos)
img.set_hshift(-0.5)
point_viz.add(img)

# Adding Label for image 1: positioned at bottom left corner
img_label = viz.Label("ARRrrr!", 0.25, 0.5)
img_label.set_rgba((1.0, 1.0, 0.0, 1))
img_label.set_scale(2)
point_viz.add(img_label)

# Adding image 2: positioned to the right of the window
img2 = viz.Image()
img_data2 = make_checker_board(10, (4, 2))
mask_data2 = np.zeros((30, 30, 4))
mask_data2[15:25, 15:25] = np.array([0, 1, 0, 0.5])
img2.set_mask(mask_data2)
img2.set_image(img_data2)
ypos2 = (0, 0.5)
xlen2 = (ypos2[1] - ypos2[0]) * img_data2.shape[1] / img_data2.shape[0]
xpos2 = (-xlen2, 0)
img2.set_position(*xpos2, *ypos2)
img2.set_hshift(1.0)
point_viz.add(img2)

# Adding Label for image 2: positioned at top left corner
img_label2 = viz.Label("Second", 1.0, 0.25, align_top=True, align_right=True)
img_label2.set_rgba((0.0, 1.0, 1.0, 1))
img_label2.set_scale(1)
point_viz.add(img_label2)


print("Added 2D images with labels, close visualizer window to continue")
point_viz.update()
point_viz.run()

# ==================================================
# Ex4: Next frame from pcap and remove some elements

point_viz.remove(img)
point_viz.remove(img_label)

img_label2.set_text("Third")

ls_viz.scan = next(scans)
ls_viz.draw()
print("Showing second frame, close visuzlier window to continue")
point_viz.run()

# won't work on macos, but convenient:
# import threading
# render_thread = threading.Thread(target=point_viz.run)
# render_thread.start()
