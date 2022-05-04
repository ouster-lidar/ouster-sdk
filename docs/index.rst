.. title:: Ouster SDK

.. toctree::
   :hidden:

   Ouster SDK Overview <self>
   Installation <installation>
   Download and Visualize Sample Data <sample-data>
   
.. toctree::
   :caption: Python Guide
   :hidden:

   Developer Quick Start <python/quickstart>
   Examples <python/examples/index>
   Point Cloud Visualizer <python/viz/index>
   Developing <python/devel>

.. toctree::
   :caption: C++ Guide
   :hidden:

   Build <cpp/building.rst>
   Examples <cpp/examples.rst>

.. toctree::
   :caption: ROS1 Guide
   :hidden:

   Build and Use <ros/index.rst>

.. toctree::
   :caption: SDK Reference
   :hidden:

   Lidar Scan API <reference/lidar-scan>
   Python API Reference <python/api/index>
   C++ API Reference <cpp/api>
   Changelog <reference/changelog>

..
   FAQ <faq>

.. toctree::
   :hidden:
   :caption: External Links

   Source Code <https://github.com/ouster-lidar/ouster_example>
   Issue Tracker <https://github.com/ouster-lidar/ouster_example/issues>
   Sensor Documentation <https://static.ouster.dev/sensor-docs/>
   More Sample Data <https://ouster.com/resources/lidar-sample-data/>


.. include:: overview.rst


All TODOs
==========

**REMOVE THIS "All TODOs" SECTION WHEN WE ARE DONE WITH DOCS!!!!**

.. note::

   Proposed versioning for the public release:

   - Python ``ouster-sdk`` is ``0.4.0``
   - C++ version in main ``ouster-sdk/CMakeLists.txt`` and a tag in Github ``ouster_example`` repo is ``v2.3.0``
   - ``ouster_client`` version in ``ouster_client/CMakeLists.txt`` is ``0.4.0``
   - bump ouster_ros to 0.3.1 or smthg

.. note::

   Future Changes to Ouster SDK that we want to announce sooner:

   - Github ``ouster_example`` repo will be renamed to ``ouster-sdk``
   - Python 3.6 support will be removed in ``ouster-sdk``
   - C++ 17 will be required for C++ and for Python bindings
   - drop Ubuntu 16.04 / ros kinetic support
   - what else?

.. todolist:: 
