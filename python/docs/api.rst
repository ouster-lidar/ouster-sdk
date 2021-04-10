=============
API Reference
=============

.. automodule:: ouster.client

Core
====

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


Metadata
========
.. autoclass:: SensorInfo
   :members:
   :undoc-members:

.. autoclass:: PacketFormat

.. autoclass:: LidarMode
   :members:
   :undoc-members:

.. autoclass:: TimestampMode
   :members:
   :undoc-members:


Data
====

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


Pcap
=====

.. automodule:: ouster.pcap

.. automodule:: ouster.pcap.pcap
   :members: info, record, Pcap, PcapInfo

