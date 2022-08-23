=======================
Point Cloud Visualizer
=======================

The Ouster visualization toolkit is written in C++ with Python bindings for Python functionality. It
consists of the following:

- ``simple-viz`` (:class:`.viz.SimpleViz`): the default Python application visualizer, which can
  also be used as an entrypoint for more sophisticated custom point cloud visualizations
- ``ouster_viz``: the core C++ library 
- :mod:`ouster.sdk.viz`: the Python module for the bindings

``simple-viz`` is a fastest way to visualize data from a connected sensor or a recorded ``pcap``:

.. figure:: /images/simple-viz.png
    :align: center

    Ouster ``simple-viz`` visualization of OS1 128 sample data

How to use ``simple-viz`` you can learn in :doc:`viz-run`

.. toctree::
   :hidden:

   viz-run


For custom visualizations you can explore a programmatically accessible :class:`.viz.PointViz` API
below:

.. figure:: /images/viz-tutorial/lidar_scan_viz_checkers.png
    :align: center

    ``PointViz`` with ``Image``, ``Label`` and masks applied


.. toctree::

   viz-api-tutorial

