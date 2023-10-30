
===========================
Module :mod:`ouster.client`
===========================

.. contents::
   :local:
   :depth: 4

.. automodule:: ouster.client


----

Core
=====

.. autoexception:: ClientError
.. autoexception:: ClientTimeout
.. autoexception:: ClientOverflow

.. autoclass:: PacketSource()
   :show-inheritance:
   :members:
   :special-members: __iter__

.. autoclass:: Packets
   :members:
   :special-members: __iter__

.. autoclass:: Sensor
   :members:
   :special-members: __iter__

.. autoclass:: Scans
   :members:
   :special-members: __iter__

----


Metadata
========

.. autoclass:: SensorInfo
   :members:
   :undoc-members:

.. autoclass:: DataFormat
   :members:

.. autoclass:: SensorConfig
   :members:
   :undoc-members:

.. autofunction:: get_config

.. autofunction:: set_config

.. autoclass:: LidarMode
   :members:
   :undoc-members:

.. autoclass:: TimestampMode
   :members:
   :undoc-members:

.. autoclass:: OperatingMode
   :members:
   :undoc-members:

.. autoclass:: MultipurposeIOMode
   :members:
   :undoc-members:

.. autoclass:: Polarity
   :members:
   :undoc-members:

.. autoclass:: NMEABaudRate
   :members:
   :undoc-members:

.. autoclass:: UDPProfileLidar
   :members:
   :undoc-members:

----


Data
=====

.. autodata:: ouster.client.data.BufferT

.. autodata:: ouster.client.data.Packet

.. autoclass:: ImuPacket
   :members:

.. autoclass:: LidarPacket
   :members:

.. autoclass:: ChanField
   :members:
   :undoc-members:

.. autoclass:: ColHeader
   :members:
   :undoc-members:

.. autoclass:: LidarScan
   :members:

.. autofunction:: XYZLut

.. autofunction:: destagger
