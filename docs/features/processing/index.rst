LidarFrame Processing
====================

After capturing lidar data, the next step is to process raw packets and frames into useful data.
The Ouster SDK provides utilities for destaggering, coordinate transforms, masking, filtering, and batch operations—enabling both real-time and offline workflows.

Key Capabilities
----------------

* **Destagger & Reproject**: Convert staggered sensor data into organized range/intensity images and rectified XYZ point clouds.
* **Lookup Table Generation**: Build reusable XYZ/RGB lookup tables for fast coordinate transforms and visualization.
* **Field Masking & Filtering**: Remove invalid returns, apply region-of-interest masks, and downsample frames for efficient processing.
* **Frame Composition**: Merge multi-sensor frames, align timestamps, and aggregate frames for mapping or SLAM.
* **Batch & Offline Processing**: Stream frames from OSF or PCAP, support windowed and indexed access, and enable parallel pipelines.

Best Practices
--------------

- **Precompute Lookup Tables**: Save and reuse XYZ/RGB tables for your sensor model and resolution to accelerate downstream tasks.
- **Validate on Samples**: Test your processing pipeline on short segments before running full batch jobs.
- **Immutable Raw Data**: Keep original OSF/PCAP files unchanged; generate processed derivatives as needed.
- **Use Vectorized Operations**: Prefer NumPy or C++ API for heavy per-point computation to maximize performance.

Find practical tips, and workflow discussions at: https://community.ouster.com/tag/sdk

.. toctree::
   :hidden:
   :maxdepth: 1

   using-cli
   using-the-api
   index-slice
   
