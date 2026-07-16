Examples Catalog
================

Welcome to the Ouster SDK examples. This section provides task-based guides
to help you get started, from connecting to a sensor and visualizing data to
building advanced mapping and C++ applications.


Python 
------

.. list-table:: Python Examples Catalog
   :header-rows: 1
   :widths: 25 35 40

   * - **Category**
     - **Description**
     - **Examples**
   * - Core module basics
     - Connect to your sensor, record data to a PCAP or OSF file. 
     - :doc:`Record and Replay Data <python/record-stream>`
   * - 
     - Working with Ouster sensor.
     - :doc:`Converting Data Formats <python/basics-sensor>`
   * - 
     - Convert PCAPs to various file formats (CSV, LAS, PCD, PLY).
     - :doc:`Converting Data Formats <python/conversion>`
   * - 
     - Read, write using OSF API.
     - :doc:`Working with OSF Files <python/osf-examples>`
   * - Visualizing Point Clouds
     - See your data. Visualize point clouds in real-time from a sensor or from a file using Open3D, and Matplotlib.
     - :ref:`Visualizing with Open3D <ex-open3d>`
   * - 
     - Visualize point clouds from a sensor or from a file using Matplotlib.
     - :ref:`Visualizing with Matplotlib <ex-visualization-with-matplotlib>`
   * - Understanding Sensor Data
     - Go deeper into the data. Understand the ``LidarFrame`` data structure. 
     - :doc:`The LidarFrame Object <python/lidar-frame>`
   * - 
     - How to work with 2D and 3D representations.
     - :ref:`2D and 3D Projections <ex-correlating-2d-and-3d>`
   * - 
     - How to parse raw UDP packets.
     - :doc:`Working with UDP Packets <python/udp-packets>`
   * - Pose Optimizer
     - Use the Pose Optimizer to refine it.
     - :doc:`Pose Optimizer </features/pose_optimizer/pose_optimizer_api>`


C++ 
---

.. list-table:: C++ Examples Catalog
   :header-rows: 1
   :widths: 25 35 40

   * - **Category**
     - **Description**
     - **Examples**
   * - C++ SDK Examples
     - Configure sensors and manage device settings directly from C++.
     - :ref:`Sensor Configuration <ex-cpp-sensor-config>`
   * - 
     - Construct ``LidarFrame`` objects and inspect their fields in code.
     - :ref:`LidarFrame constructors <ex-cpp-lidarframe>`
   * - 
     - Destagger frames and generate 2D/3D representations for visualization.
     - :ref:`2D & 3D representations <ex-cpp-representations>`
   * - Linking the SDK
     - Learn how to link against the shared or static library.
     - :doc:`C++ Linking Examples <cpp/linking_examples>`


Coming Soon
-----------

We are actively expanding the examples catalog with additional, scenario-driven
walkthroughs (ROS integrations, pose optimizer, zone monitor, perception, and ground segmentation). 

If there is a workflow you would like to see documented, please reach
out on the `Ouster Community Forum <https://community.ouster.com/tag/sdk>`_.

Stay tuned—new tutorials will appear here soon.


.. toctree::
   :maxdepth: 1
   :class: is-index
   :caption: Examples

   python/index
   cpp/index