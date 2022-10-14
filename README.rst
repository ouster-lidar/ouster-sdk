.. figure:: https://github.com/ouster-lidar/ouster_example/raw/master/docs/images/Ouster_Logo_TM_Horiz_Black_RGB_600px.png

------------------------------------------------------

=========================================================
Ouster SDK - libraries and tools for Ouster Lidar Sensors
=========================================================

Cross-platform C++/Python Ouster Sensor Development Toolkit

To get started with our sensors, client, and visualizer, please see our SDK and sensor documentation:

- `Ouster SDK Documentation <https://static.ouster.dev/sdk-docs/index.html>`_
- `Ouster Sensor Documentaion <https://static.ouster.dev/sensor-docs>`_ 

This repository contains Ouster SDK source code for connecting to and configuring ouster sensors,
reading and visualizing data.

* `ouster_client <ouster_client/>`_ contains an example C++ client for ouster sensors
* `ouster_pcap <ouster_pcap/>`_ contains C++ pcap functions for ouster sensors
* `ouster_viz <ouster_viz/>`_ contains a customizable point cloud visualizer
* `python <python/>`_ contains the code for the ouster sensor python SDK (``ouster-sdk`` Python package)

.. note::
    Ouster ROS driver code has been moved out to a separate GitHub repository. To get started using the
    driver follow the instructions provided on the repository's main page: https://github.com/ouster-lidar/ouster-ros


License
=======

BSD 3-Clause License, `details <LICENSE>`_
