==================
C++ SDK Reference
==================

.. todo::

   C++ SDK reference Section is not complete. Structure, content, etc...
   BTW, we can reference methods and objects similar to the ``:py:`` domain, though not as flexible:

      - :cpp:class:`ouster::LidarScan` for class
      - :cpp:func:`ouster::stagger()` for function
      - :cpp:enumerator:`ouster::sensor::lidar_mode::MODE_512x10` for enum value
      - :cpp:enum:`ouster::sensor::lidar_mode` for enums

.. contents::
   :local:
   :depth: 4


Library/Target ``ouster_client``
================================


``ouster/types.h``
------------------

.. doxygenfile:: ouster/types.h


``ouster/lidar_scan.h``
------------------------

.. doxygenfile:: ouster/lidar_scan.h


``ouster/client.h``
--------------------

.. doxygenfile:: ouster/client.h


``ouster/image_processing.h``
------------------------------

.. doxygenfile:: ouster/image_processing.h



Library/Target ``ouster_pcap``
===============================

``ouster/os_pcap.h``
---------------------

.. doxygenfile:: ouster/os_pcap.h


Library/Target ``ouster_viz``
==============================

.. doxygennamespace:: ouster::viz
   :members:
   :undoc-members:
   :protected-members:




