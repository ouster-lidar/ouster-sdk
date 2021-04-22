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

.. autoclass:: Packets
   :members:

.. autoclass:: Sensor
   :members:

.. autoclass:: Scans
   :members:

----

Metadata
--------

.. autoclass:: SensorInfo
   :members:
   :undoc-members:

.. autoclass:: PacketFormat

.. autoclass:: SensorConfig
   :members:
   :undoc-members:

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

Example :mod:`ouster.sdk.examples.client`
=========================================

.. automodule:: ouster.sdk.examples.client
   :members:


Example :mod:`ouster.sdk.examples.pcap`
========================================

.. automodule:: ouster.sdk.examples.pcap
   :members:

