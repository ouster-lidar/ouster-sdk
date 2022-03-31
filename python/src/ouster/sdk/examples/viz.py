"""Example of interactive visualizer use.

Intended to run with `python -i -m ouster.sdk.examples.viz`

TODO:
- separate set scan / update is annoying
- a proxy run()/quit() on ls_viz would be useful
- maybe: ls_viz could initialize underlying viz + expose it
- point_viz.run() twice is broken
- ideally, run() would open/close window
- auto camera movement example?
"""

from ouster import client, pcap
from ouster.sdk import viz

meta_path = "/mnt/aux/test_drives/OS1_128_2048x10.json"
pcap_path = "/mnt/aux/test_drives/OS1_128_2048x10.pcap"

meta = client.SensorInfo(open(meta_path).read())
packets = pcap.Pcap(pcap_path, meta)
scans = iter(client.Scans(packets))

point_viz = viz.PointViz("Example Viz")
ls_viz = viz.LidarScanViz(meta, point_viz)

ls_viz.scan = next(scans)
ls_viz.draw()
print("Showing first frame, close visuzlier window to continue")
point_viz.run()

ls_viz.scan = next(scans)
ls_viz.draw()
print("Showing second frame, close visuzlier window to continue")
point_viz.run()

# won't work on macos, but convenient:
# import threading
# render_thread = threading.Thread(target=point_viz.run)
# render_thread.start()
