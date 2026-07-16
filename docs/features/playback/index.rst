Recording & Playback
====================

After configuring the sensor settings and learning how to consume live streams,
we can capture lidar data for reproducible testing, shareable datasets, or long-term logging.

The Ouster SDK bundles recorders, converters, and playback utilities that operate on the
same abstractions used in previous sections.

Key Capabilities
----------------

* **One-Command Capture**: Record live sensors to OSF or PCAP while simultaneously saving metadata.
* **Deterministic Playback**: Replay archived frames for regression tests and reproducibility.
* **Format Conversion**: Translate captures into interchange formats (PLY, LAS, ROS bag, CSV) for downstream processing.
* **Granular Control**: Trim, filter, or downsample datasets without touching the original raw files.

Best Practices
--------------

- **Capture Metadata Together**: Always save sensor metadata alongside PCAP files for accurate replay.
- **Record OSF for Rich Workflows**: OSF is recommended for advanced workflows, indexing, and multi-stream support.
- **Log Environment State**: Note calibration, firmware, and network setup at record time to simplify debugging later.
- **Verify Replay Early**: Immediately replay a short segment to confirm the capture is usable before leaving the site.
- **Segment Large Jobs**: Split recordings by duration or disk size so processing and transfer stay manageable.
- **Use Filters at Replay Time**: Keep the raw dataset intact; apply `clip`, `filter`, or `mask` during playback to generate derivatives.


.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: How-to articles

   using-cli
   using-the-api
   using-the-viz
