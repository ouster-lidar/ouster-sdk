============================
Module :mod:`ouster.sdk.osf`
============================

.. contents::
   :local:
   :depth: 4

.. automodule:: ouster.sdk.osf

----

Low-Level API
-------------

Writing OSF files
^^^^^^^^^^^^^^^^^

``Writer`` to create OSF file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: Writer
   :members:
   :undoc-members:


High-Level API
--------------

``osf.Scans`` just read ``LidarScan`` objects from a file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: Scans
   :members:
   :special-members: __iter__
