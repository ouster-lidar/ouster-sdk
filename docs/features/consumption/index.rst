Data Consumption
================

The Ouster SDK's data consumption layer provides a flexible framework for building robust data pipelines.
These guides show you how to read lidar data from live sensors or recorded files, manage multi-sensor streams,
and prepare data for downstream applications like visualization, mapping, and analytics.


Quickstart
----------

The fastest way to get started reading data is the :doc:`command line <using-cli>`. ``ouster-cli source``
accepts the various inputs covered throughout this section — a sensor hostname, a ``.pcap``, or an ``.osf`` file —
with no code required.

Under the hood the CLI uses :ouster:func:`open_source <py=ouster.sdk.open_source|cpp=ouster::sdk::open_source>`.
To understand and use that API directly from Python or C++, read the guides below. You can also visualize the
data interactively — see :doc:`using-the-viz`.

A brief description of the sections for API users:

* :doc:`frame_set_sources` — the source types and the ``open_source`` helper (start here for concepts).
* :doc:`using-the-api` — reading frames and packets in depth: parameters, iteration, and metadata.
* :doc:`lidar_frame` — working with frames, fields, headers, and the batcher.
* :doc:`collation` — how multi-sensor frames are synchronized into a ``FrameSet``.
* :doc:`field` — low-level C++ ``Field`` and ``ArrayView`` access.


Core concepts
-------------

A few objects appear throughout these guides:

* :ouster:class:`PacketSource <py=ouster.sdk.core.PacketSource|cpp=ouster::sdk::core::PacketSource>` — a low-level
  iterator that yields raw ``LidarPacket`` and ``ImuPacket`` objects straight from a sensor or file.
* :ouster:class:`FrameSetSource <py=ouster.sdk.core.FrameSetSource|cpp=ouster::sdk::core::FrameSetSource>` — the
  high-level iterator returned by ``open_source``. It batches packets into complete frames and yields ``FrameSet`` objects.
* :ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>` — a container holding one
  ``LidarFrame`` per sensor for a given time slice, so iterating a source yields ``FrameSet`` objects you index into.
  See :doc:`frame_set_sources` to learn more.
* :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` — one sensor's full
  rotation, with all channel fields (range, signal, reflectivity, …) and per-column headers.


Key Capabilities
----------------

* **Unified Abstraction**: Use a single interface to process data from live sensors, PCAP, OSF and ROSBAG files.
* **Multi-Sensor Support**: Easily ingest and align data from multiple sensors for synchronized pipelines.
* **Optimized Performance**: The SDK handles low-level tasks like packet batching, allowing you to focus on your application.


Best Practices
--------------

- **Stream-Process Frames:** Process your data stream inline instead of buffering large lists of ``LidarFrame`` objects.
- **Monitor Dropped-Frame and ID Counters:**  Non-zero values usually indicate packet loss or stale metadata.
- **Avoid unnecessary collation:** Collate only when you need synchronized multi-sensor output; skip it for single-sensor paths to reduce CPU work.
- **Separate I/O and Compute Threads:** Isolate I/O from heavy compute so packet ingestion is not blocked by downstream processing.
- **Maintain Separate Per-Sensor State:** Keep per-sensor state separate to avoid assumptions about frame shape in multi-sensor pipelines.

Troubleshooting
---------------

- **Connection / Stream Not Starting:**
  
  - Check hostname/IP reachability (ping). 
  - Confirm correct metadata (JSON) for replay; mismatch -> malformed frames.

- **Incomplete or Partial Frames:**
  
  - Cause: UDP loss or mode/profile change mid-stream.
  - Action: Lower data rate (lidar_mode, dual returns off, narrow azimuth_window). Monitor packet drops.

- **Frame ID Resets Unexpectedly:**
  
  - Occurs after sensor reconfig or reboot. Refresh SensorInfo and rebuild XYZ LUT.

- **Timestamps Drift Across Sensors:**
  
  - External sync not locked. Use same timesync_mode (PTP/GPS) and verify stability before multi-source collation.

- **Dual Returns Missing RANGE2:**
  
  - Profile not applied or sensor FW unsupported. Re-read config; ensure udp_profile_lidar dual profile active.

- **Out-of-Order Multi-Sensor Bundles:**
  
  - Different timing sources. Align by timestamp after enabling common sync mode.

.. toctree::
   :hidden:
   :maxdepth: 1

   using-cli
   frame_set_sources
   using-the-api
   lidar_frame
   collation
   field
   using-the-viz
   
