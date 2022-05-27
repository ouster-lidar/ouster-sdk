Ouster Sensor SDK
=================

The Ouster Sensor SDK provides developers interfaces for interacting with sensor hardware and
recorded sensor data suitable for prototyping, evaluation, and other non-safety-critical
applications in Python and C++. Example and reference code is provided for common operations on
sensor data in both languages. The SDK includes APIs for:

* Querying and setting sensor configuration
* Recording and reading data in pcap format
* Reading and buffering sensor UDP data streams reliably
* Conversion of raw data to range/signal/near_ir/reflectivity images (destaggering)
* Efficient projection of range measurements to Cartesian (x, y, z) corrdinates
* Visualization of multi-beam flash lidar data

Additionally, in Python, the SDK also provides: 

* Frame-based access to lidar data as numpy datatypes
* A responsive visualizer utility for pcap and sensor

Quick links
-----------

* :doc:`installation`
* :doc:`python/quickstart`
* :doc:`cpp/building`
* :doc:`ros/index`

.. figure:: /images/simple-viz.png
    :align: center

* :doc:`python/viz/index`

Status and Contact Info
-----------------------

The Ouster SDK is currently provided as an early pre-1.0 preview. The APIs are subject to change in
every release.

For questions about using your Ouster hardware, you may find it useful to reference the `Ouster
sensor documentation`_ and/or contact `Ouster support`_. For issues specific to the SDK please use
the `GitHub issue tracker`_.

.. _Ouster sensor documentation: https://static.ouster.dev/sensor-docs/index.html
.. _Ouster support: https://ouster.atlassian.net/servicedesk/customer/portal/8
.. _Github issue tracker: https://github.com/ouster-lidar/ouster_example/issues
