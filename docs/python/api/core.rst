Module :mod:`ouster.sdk.core`
=============================

.. rubric:: ouster.sdk.core.clipped_scan_source

.. automodule:: ouster.sdk.core.clipped_scan_source
   :members:
   :undoc-members:
   :show-inheritance:

.. rubric::  ouster.sdk.core

Functions
---------

.. autofunction:: ouster.sdk.core.collate
.. autofunction:: ouster.sdk.core.resolve_field_types
.. autofunction:: ouster.sdk.core.io_type
.. autofunction:: ouster.sdk.core.io_type_from_extension
.. autofunction:: ouster.sdk.core.extension_from_io_type
.. autofunction:: ouster.sdk.core.populate_extrinsics
.. autofunction:: ouster.sdk.core.euler_pose_to_matrix
.. autofunction:: ouster.sdk.core.quaternion_pose_to_matrix
.. autofunction:: ouster.sdk.core.interp_pose
.. autofunction:: ouster.sdk.core.voxel_downsample
.. autofunction:: ouster.sdk.core.read_pointcloud
.. autofunction:: ouster.sdk.core.dewarp
.. autofunction:: ouster.sdk.core.transform


Classes
-------

.. autoclass:: ouster.sdk.core.Singler
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: ouster.sdk.core.Slicer
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: ouster.sdk.core.Collator
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: ouster.sdk.core.PacketSource
   :show-inheritance:
   :members:
   :special-members: __iter__

.. autoclass:: ouster.sdk.core.SensorInfo
   :members:
   :special-members: __iter__

.. autoclass:: ouster.sdk.core.SensorPacketSource
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: ouster.sdk.core.ScanSource
   :members:
   :special-members: __iter__
   :undoc-members:
   :show-inheritance:

Deprecation warnings: 

- ``ScanSource.sensors_count`` is deprecated.  To get the number of sensors get the length of sensor_info.  
- ``ScanSource.metadata`` is deprecated. To get the sensor_info for each sensor use ``sensor_info`` instead.
  
  ``sensors_count()`` and ``metadata`` will be removed in future releases.


.. autoclass:: ouster.sdk.core.SensorScanSource
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: ouster.sdk.core.LidarScan
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __iter__

.. autoclass:: ouster.sdk.core.ImuPacket
   :members:

.. autoclass:: ouster.sdk.core.LidarPacket
   :members:

.. autoclass:: ouster.sdk.core.UDPProfileLidar
   :members:
   :undoc-members:


.. rubric:: ouster.sdk.core.core

.. automodule:: ouster.sdk.core.core
   :members:
   :undoc-members:
   :show-inheritance:

.. rubric:: ouster.sdk.core.data

.. automodule:: ouster.sdk.core.data
   :members:
   :undoc-members:
   :show-inheritance:

.. rubric:: ouster.sdk.core.io_types

.. automodule:: ouster.sdk.core.io_types
   :members:
   :undoc-members:
   :show-inheritance:

.. rubric:: ouster.sdk.core.masked_scan_source

.. automodule:: ouster.sdk.core.masked_scan_source
   :members:
   :undoc-members:
   :show-inheritance:

.. rubric:: ouster.sdk.core.multi

.. deprecated::
   collate_scans is deprecated: use ouster.sdk.core.collate instead. collate_scans will be removed in the upcoming release.

.. automodule:: ouster.sdk.core.multi
   :members:
   :undoc-members:
   :show-inheritance:

.. rubric:: ouster.sdk.core.reduced_scan_source

.. automodule:: ouster.sdk.core.reduced_scan_source
   :members:
   :undoc-members:
   :show-inheritance:

.. rubric:: ouster.sdk.core.scan_ops

.. automodule:: ouster.sdk.core.scan_ops
   :members:
   :undoc-members:
   :show-inheritance:


Enums
-----

.. autoenum:: ouster.sdk.core.IoType
.. autoenum:: ouster.sdk.core.PacketType
.. autoenum:: ouster.sdk.core.PacketValidationFailure
.. autoenum:: ouster.sdk.core.ClientState
.. autoenum:: ouster.sdk.core.FieldClass
