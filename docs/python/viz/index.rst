======================
Point Cloud Visualizer
======================

The Ouster visualization toolkit is written in C++ with Python bindings for Python functionality. It
consists of the following:

- ``simple-viz`` (:class:`.viz.SimpleViz`): the default Python application visualizer, which can
  also be used as an entrypoint for more sophisticated custom point cloud visualizations
- ``ouster_viz``: the core C++ library 
- :mod:`ouster.viz`: the Python module for the bindings

.. todo::

    Update all ``simple-viz`` CLI command mentions to a proper new CLI command + update the
    screenshots to the current looking version

Using ``ouster-cli`` is a fastest way to visualize data from a connected sensor, recorded ``pcap``
or OSF files with SLAM poses:

.. figure:: /images/ouster-viz.png
    :align: center

    Ouster SDK CLI ``ouster-cli source OS-1-128.pcap viz`` visualization of OS1 128 sample data

How to use ``ouster-cli`` for visualizations you can learn in :doc:`viz-run`

.. toctree::
   :hidden:

   viz-run


For custom visualizations you can explore a programmatically accessible :class:`.viz.PointViz` API
below:

.. figure:: /images/viz-tutorial/lidar_scan_viz_checkers.png
    :align: center

    ``PointViz`` with ``Image``, ``Label`` and masks applied


.. toctree::

   Visualize SLAM Poses <viz-scans-accum>
   viz-api-tutorial

