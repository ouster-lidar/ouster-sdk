Python
======

.. _examples-setup:

A loosely connected collection of examples and concepts useful for working with the Ouster Python
SDK. If you are just starting, please see :doc:`/../getting-started/installation`.

To start developing with source code, please see :doc:`/../development/python`.

For convenience, we will use ``$SAMPLE_DATA_PCAP_PATH`` and ``$SAMPLE_DATA_JSON_PATH`` for the locations of the sample data pcap
and json.

For Python code, ``pcap_path`` and ``sensor_info_path`` are taken to be set to those values,
respectively:

.. code:: python

   pcap_path = '<SAMPLE_DATA_PCAP_PATH>'
   sensor_info_path = '<SAMPLE_DATA_JSON_PATH>'

Similarly, ``$SENSOR_HOSTNAME`` is used for your sensor's hostname.


.. list-table:: Python Examples Catalog
   :header-rows: 1
   :widths: 25 35 40

   * - **Category**
     - **Description**
     - **Examples**
   * - Core module basics
     - Connect to your sensor, record data to a PCAP or OSF file. 
     - :doc:`Record and Replay Data <record-stream>`
   * - 
     - Working with Ouster sensor.
     - :doc:`Converting Data Formats <basics-sensor>`
   * - 
     - Convert PCAPs to various file formats (CSV, LAS, PCD, PLY).
     - :doc:`Converting Data Formats <conversion>`
   * - 
     - Read, write using OSF API.
     - :doc:`Working with OSF Files <osf-examples>`
   * - Visualizing Point Clouds
     - See your data. Visualize point clouds in real-time from a sensor or from a file using Open3D, and Matplotlib.
     - :ref:`Visualizing with Open3D <ex-open3d>`
   * - 
     - Visualize point clouds from a sensor or from a file using Matplotlib.
     - :ref:`Visualizing with Matplotlib <ex-visualization-with-matplotlib>`
   * - Understanding Sensor Data
     - Go deeper into the data. Understand the ``LidarFrame`` data structure. 
     - :doc:`The LidarFrame Object <lidar-frame>`
   * - 
     - How to work with 2D and 3D representations.
     - :ref:`2D and 3D Projections <ex-correlating-2d-and-3d>`
   * - 
     - How to parse raw UDP packets.
     - :doc:`Working with UDP Packets <udp-packets>`
   * - Pose Optimizer
     - Use the Pose Optimizer to refine it.
     - :doc:`Pose Optimizer </features/pose_optimizer/pose_optimizer_api>`



.. toctree::
   :hidden:
   :maxdepth: 1

   lidar-frame
   record-stream
   udp-packets
   osf-examples
   visualizations
