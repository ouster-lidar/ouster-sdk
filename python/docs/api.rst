=============
API Reference
=============


Module :mod:`ouster.client`
===========================

.. contents::
   :local:
   :depth: 4

.. automodule:: ouster.client


----

Core
----

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
--------

.. autoclass:: SensorInfo
   :members:
   :undoc-members:

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

----


Data
----

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


----

Module :mod:`ouster.pcap`
=========================

.. automodule:: ouster.pcap

.. autofunction:: info

.. autoclass:: PcapInfo
   :members:

.. autofunction:: record

.. autoclass:: Pcap
   :members:


----

Module :mod:`ouster.sdk.examples`
=================================

.. contents::
   :local:
   :depth: 4

.. automodule:: ouster.sdk.examples

----


Client Examples :mod:`ouster.sdk.examples.client`
-------------------------------------------------

.. automodule:: ouster.sdk.examples.client
   :members:

----


PCAP Examples :mod:`ouster.sdk.examples.pcap`
---------------------------------------------

.. automodule:: ouster.sdk.examples.pcap
   :members:

----


Open3D Examples :mod:`ouster.sdk.examples.open3d`
-------------------------------------------------

.. automodule:: ouster.sdk.examples.open3d
   :members:

----


Reference Code :mod:`ouster.sdk.examples.reference`
---------------------------------------------------

.. automodule:: ouster.sdk.examples.reference
   :members:
