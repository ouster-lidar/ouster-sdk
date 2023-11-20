=================
SLAM API Examples
=================

.. contents::
   :local:
   :depth: 3

.. _slam-api-example:

This guide provides examples of how to use the SLAM API with visualizer and obtain the sensor orientation.
Users can run SLAM with an OS sensor's IP for real time processing or with a recorded PCAP/OSF
file for offline processing.


Extract Scans and Metadata
==========================
We will demonstrate how to extract lidar scan objects and sensor's metadata from different sources which
will be served as the input of the SLAM algrotihm

.. code:: python

   from ouster import client
   from ouster.pcap import Pcap
   import ouster.osf as osf
   from ouster.sdkx.parsing import default_scan_fields
   from ouster.sdk.util import resolve_metadata
   import ipaddress

   def load_source(source):
       if source.endswith('.pcap'):
           metadata = open(resolve_metadata(source), "r").read()
           info = client.SensorInfo(metadata)
           pcap = Pcap(source, info)
           fields = default_scan_fields(info.format.udp_profile_lidar, flags=True)
           scans = client.Scans(pcap, fields=fields)
       elif source.endswith('.osf'):
           scans = osf.Scans(source)
           info = scans.metadata
       elif source.endswith('.local') or ipaddress.ip_address(source):
           scans = client.Scans.stream(hostname=source, lidar_port=LIDAR_PORT, complete=False, timeout=1)
           info = scans.metadata
       else:
           raise ValueError("Not a valid file type")

       return scans, info

Obtain Lidar Pose
=================
The SLAM API outputs the sensor's pose for each lidar scan, which you can use to determine the
sensor's orientation in your system.

.. code:: python

   from ouster import client
   from ouster.mapping.slam import KissBackend
   scans, info = load_source(pcap_path)
   slam = KissBackend(info, max_range=75, min_range=1, voxel_size=1.0)

   for idx, scan in enumerate(scans):
        scan_w_poses = slam.update(scan)
        col = client.first_valid_column(scan_w_poses)
        print(f"idx = {idx} and pose {scan_w_poses.pose[col]}")


SLAM with Visulizer and Accumulated Scans
=========================================
Visualizers and Accumulated Scans are also available for monitoring the performance of the algorithm,
as well as for demonstration and feedback purposes.

.. code:: python

   from functools import partial
   from ouster.viz import SimpleViz, ScansAccumulator
   from ouster.mapping.slam import KissBackend
   scans, info = load_source(pcap_path)
   slam = KissBackend(info, max_range=75, min_range=1, voxel_size=1.0)

   scans_w_poses = map(partial(slam.update), scans)
   scans_acc = ScansAccumulator(info,
                                accum_max_num=10,
                                accum_min_dist_num=1,
                                map_enabled=True,
                                map_select_ratio=0.01)

   SimpleViz(info, scans_accum=scans_acc, rate=0.0).run(scans_w_poses)

More details about the visualizer and accumulated scans can be found at the
:ref:`Ouster Visualizer <viz-run>` and :ref:`Scans Accumulator <viz-scans-accum>`


.. note::

   The performance of the SLAM algorithm depends on your CPU's processing power and the 'voxel_size'
   parameter.
   Below is a suggestion for selecting an appropriate voxel size:

   | Outdoor: 1.4 - 2.2
   | Large indoor: 1.0 - 1.8
   | Small indoor: 0.4 - 0.8
