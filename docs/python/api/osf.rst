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

Reading
^^^^^^^

``Reader`` for OSF files
~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: Reader
   :members:
   :undoc-members:


``MessageRef`` wrapper for a `message`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: MessageRef
   :members:
   :undoc-members:

``MetadataStore`` for `metadata entries`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: MetadataStore
   :members:
   :undoc-members:

``MetadataEntry`` base class for all metadata
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: MetadataEntry
   :members:
   :undoc-members:


Writing OSF files
^^^^^^^^^^^^^^^^^

``Writer`` to create OSF file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: Writer
   :members:
   :undoc-members:


Common `metadata entries`
^^^^^^^^^^^^^^^^^^^^^^^^^

``LidarSensor`` Ouster sensor metadata (i.e. ``client.SensorInfo``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: LidarSensor
   :members:
   :undoc-members:

``StreamStats`` statistics per stream
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: StreamStats
   :members:
   :undoc-members:

``StreamingInfo`` stream statistics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: StreamingInfo
   :members:
   :undoc-members:


Common `streams`
^^^^^^^^^^^^^^^^

``LidarScanStream`` stream
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: LidarScanStream
   :members:
   :undoc-members:


High-Level API
--------------

``osf.Scans`` just read ``LidarScan`` objects from a file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: Scans
   :members:
   :special-members: __iter__
