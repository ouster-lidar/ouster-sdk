.. _osf:

==================
Open Sensor Format
==================

The *Open Sensor Format* (OSF) is an extensible file format for storing
time-series data, based on FlatBuffers_.

Compared to pcap, it offers the following advantages:

- Messages can more easily be randomly-accessed because an index from message
  timestamp to the file offset of the chunk containing the message is contained
  within the file.
- Full frames of lidar data are meant to be compressed as individual channels
  (using lossless PNG-based compression by default,) resulting in smaller file
  sizes for the same data.
- Sensor configuration is contained within the file.

The Ouster SDK provides both Python and C++ methods for working with OSF files.

Reading and writing OSF files
-----------------------------

Reading, writing, and recording new OSF files are all possible with the Ouster
SDK and the Ouster SDK CLI.

* :doc:`Python API examples <../python/examples/osf-examples>`
* :doc:`C++ API examples <../cpp/examples/simple_examples>`
* :doc:`CLI examples <../cli/sample-sessions>`

Getting example OSF files
-------------------------

The :doc:`sample data page <../sample-data>` has instructions for obtaining sample datasets. Sample
datasets are availble in OSF format in addition to pcap.

OSF format details
------------------

Typically, users of the Ouster SDK won't have to worry about the implementation
details of OSF. The following is a very basic overview of the structure of an
OSF file.

An OSF file generally contains the following:

#. A header, which contains the location of the OSF metadata.
#. A series of "chunks", each of which contain one or more messages, typically containing lidar scans - see :ref:`LidarScan reference <lidar-scan>`.

#. Metadata, a collection of generic buffers usually containing the following:

   a. An index of chunks (meant to provide the file offset, in bytes, for a chunk given a timestamp.)
   b. An ``ouster/v1/streaming/StreamingInfo``, which contains the start, end timestamps and number of messages for each chunk as well as some statistics.
   c. An ``ouster/v1/os_sensor/LidarSensor``, which contains the configuration of the sensor used to collect the data contained in a stream of lidar scans.
   d. An ``ouster/v1/os_sensor/LidarScanStream``, which indicates which fields are present in each lidar scan.

For more details of the structure of an OSF file, consult the definition files
found in the ``ouster_osf/fb`` directory of the Ouster SDK repository.

.. _FlatBuffers: https://flatbuffers.dev/
