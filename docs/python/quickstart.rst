.. _quickstart:

==================================================
Developer's Quick Start with the Ouster Python SDK
==================================================

.. contents::
   :local:
   :depth: 3

This quickstart guide will walk you through visualizing Ouster sensor data quickly with Python code
you write yourself. It assumes that you have followed the steps in :ref:`Python Installation
<installation-python>` to install the Ouster Python SDK.


Using this Guide
================

You'll want to start an interactive Python session and keep it open through the sections, as we'll
be reusing variables created in earlier parts while explaining what we're doing as we go.

To get started, open a new console/Powershell window and start a python interpreter:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3

    .. code-tab:: powershell Windows x64

        PS > py -3


Throughout this guide we will indicate console commands with ``$`` and python interpreter commands
with ``>>>``, just as we have above.

If you'd like to start by working with sample data, continue to the next section. If you'd prefer to
start by capturing data from a sensor, you can skip to `Using an Ouster Sensor`_ below.


Using Sample Data
=================

.. include:: /sample-data.rst
    :start-after: [start-download-instructions]
    :end-before: [end-download-instructions]

The downloaded pcap file contains lidar and imu packets captured from the network. You can read
more about the `IMU Data Format`_ and `Lidar Data Format`_ in the Ouster Sensor Documentation. The
JSON file contains metadata queried from the sensor TCP interface necessary for interpreting
the packet data.

In your open python session, save the two paths to variables:

.. code:: python

   >>> pcap_path = '<SAMPLE_DATA_PCAP_PATH>'
   >>> metadata_path = '<SAMPLE_DATA_JSON_PATH>'

.. note::
   Starting with ouster-sdk v0.11.0, most of core sdk objects have been moved from the ``ouster``
   namespace into the ``ouster.sdk`` namespace.


Using the ScanSource interface
==============================

.. _scan-source-example:

In this example we are going to demonstrate the use of the ScanSource API.


Using the open_source method
----------------------------

The ``ScanSource`` API introduces a new method name ``open_source`` which allows users to handle different source types
using the same API. Current supported source types are live sensor, pcap file, or osf file.
For example, opening the sample pcap file can be accomplished as follows:

.. code:: python

   >>> pcap_path = '<SAMPLE_DATA_PCAP_PATH>'
   >>> metadata_path = '<SAMPLE_DATA_JSON_PATH>'
   >>> from ouster.sdk import open_source
   >>> source = open_source(pcap_path, meta=[metadata_path])


The ``source`` object returned by ``open_source`` provides access to ``LidarScan`` objects,
regardless of whether the source data comes from a sensor, pcap, or osf file.

Notice here that rather than we try to load and parse the metadata ourselves we only need to pass to metadata
to the method through ``meta`` parameter and the method will take care of loading it and associating it with the
source object. The ``meta`` parameter however is optional and can be omitted. When the ``meta`` parameter is not
set explicitly the ``open_source`` method will attempt to locate the metadata automatically for us and we can reduce
the call to:

.. code:: python

   >>> source = open_source(pcap_path)

However if metadata file is not in the same folder as the pcap and don't have a shared name prefix the method will
fail.


.. note::
   Another optional but important parameter for the ``open_source`` method is ``sensor_idx``. This parameter is set to
   zero by default, which should always be the case unless the pcap file that you are using (or osf or any LidarScan
   storage) contains scans from more than one sensor, in this case, users can set the ``sensor_idx`` to a any value
   between zero and ``sensors_count -1`` to access and manipulate scans from a specific sensor by the order they appear
   in the file. Alternatively, if users set the value of ``sensor_idx`` to ``-1`` then ``open_source`` will return a
   slightly different interface from ``ScanSource`` which is the ``MultiScanSource`` this interface and as the name
   suggests allows users to work with sensor data collected from multiple sensors at the same time.

   The main different between the ``MultiScanSource`` and the ``ScanSource`` is the expected return of some of the object
   methods. For example, when creating an iterator for a ``ScanSource`` object, the user would get a single ``LidarScan``
   object per iteration. Iterating over the contents of a ``MultiScanSource`` object always yields a **list** of
   ``LidarScan(s)`` per iteration corresponding to the number of sensors stored in the pcap file or whatever source type
   is being used. This is true even when the pcap file contains data for a single sensor.


On the other hand, if the user wants to open an OSF file or access a live sensor, all that changes is URL
of the source. For example, to interact with a live sensor the user can execute the following snippet:

.. code:: python

   >>> sensor_url = '<SENSOR-HOSTNAME-OR-IP>'
   >>> from ouster.sdk import open_source
   >>> source = open_source(sensor_url)


Obtaining sensor metadata
-------------------------

Every ScanSource holds a reference to the sensor metadata, which has crucial information that is important when
when processing the individual scans. Continuing the example, a user this can access the metadata through the
``metadata`` property of a ``ScanSource`` object:

.. code:: python

   >>> print(source.metadata)


Iterating over Scans
--------------------

Once we have successfully obtain a handle to the ScanSource we can iterate over ``LidarScan`` objects stored in the
pcap file and manipulate each one individually. For example, let's say we want to print the frame id of the first 10
scans. We can achieve that using:

.. code:: python

   >>> ctr = 0
   >>> source_iter = iter(source)
   >>> for scan in source_iter:
   ...     print(scan.frame_id)
   ...     ctr += 1
   ...     if ctr == 10:
   ...         break

As we noted earlier, if we set ``sensor_idx=-1`` when invoking ``open_source`` method, the method will construct a
``MultiScanSource``, which always addresses a group of sensors. Thus, when iterating over the ``source`` the user
receives a collated set of scans from the addressed sensors per iteration. The ``MultiScanSource`` examines the
timestamp of every scan from every sensor and returns a list of scans that fit within the same time window as single
batch. The size of the batch is fixed corresponding to how many sensors contained in the pcap or OSF file. However,
the collation could yield a null value if one or more of the sensors didn't produce a ``LidarScan`` object that fits
within the time frame of current batch or iteration. Thus, depending on the operation at hand it is critical to check
if we got a valid ``LidarScan`` object when examining the iteration output of a ``MultiScanSource``.  If we are to
perform the same example as above when ``source`` is a handle to ``MultiScanSource`` and display the frame_id of
``LidarScan`` objects the belongs to the same batch on the same line the code needs to updated to the following:

.. code:: python

   >>> ctr = 0
   >>> source_iter = iter(source)
   >>> for scans in source_iter:
   ...     for scan in scans:    # source_iter here returns a list of scans
   ...         if scan:          # check if individual scan object is valid
   ...             print(scan.frame_id, end=', ')
   ...     print()   # new line for next batch
   ...     ctr += 1
   ...     if ctr == 10:
   ...         break


Note that when iterating over a ``MultiScanSource`` object, it always a list of scans, even when the underlying scan
source has only a single sensor. In this case, the iterator will yield a list with a single element per iteration.


Using indexing and slicing capabilities of a ScanSource
-------------------------------------------------------

One of the most prominent new features of the ScanSource API, (besides being able to address multi sensors), is the
ability to use indexing and slicing when accessing the stored scans within the ``LidarScan`` source. Currently, this
capability is only supported for indexable sources. That is to say, the functionality we are discussing can only be used
when accessing a pcap or an OSF file with indexing turned on. To turn on indexing simply add the ``index`` flag and set
it ``True`` when opening a pcap or OSF file:


.. code:: python

   >>> pcap_path = '<SAMPLE_DATA_PCAP_PATH>'
   >>> from ouster.sdk import open_source
   >>> source = open_source(pcap_path, index=True)

.. note::
   We omitted the ``meta`` parameter since it can be populated automatically as we explained earlier.

Depending on the file size and the underlying file format there can be some delay before the file is fully indexed (OSF
file take much less time than pcap file to index). A progress bar will appear to indicate progress of the indexing.

Once the index is built up, then we can start using utilizing and interact with the ``ScanSource`` object to access scans
in the same manner we are dealing with a python list that holds reference to LidarScan objects.

For example to access the 10th LidarScan and print its frame id, we can do the following:

.. code:: python

   >>> print(source[10].frame_id)

Similarly we can access the last LidarScan object and print its frame_id using:

.. code:: python

   >>> print(source[-1].frame_id)


Alternatively we can instead request a range of scans using the python slice operator. For example, to request the first 10
scans from a ScanSource and print their frame ids, we can do the following:

.. code:: python

   >>> for scan in source[0:10]:
   ...     print(scan.frame_id)


Note we don't need to add any break here since the operation `source[0:10]` will only yield the first 10 ``LidarScan(s)``.

To print frame_id of the last 10 LidarScans we do:

.. code:: python

   >>> for scan in source[-11:-1]:
   ...     print(scan.frame_id)


Finally, as you would expect from a typical slice operation you can also use a step value, though reversed
iteration is not supported.

.. code:: python

   >>> for scan in source[0:10:2]:     # prints the frame_id of every second scan of the first 10 scans
   ...     print(scan.frame_id)

   >>> for scan in source[10:0:-1]:    # unsupported
   ...     print(scan.frame_id)


Slicing operator as a ScanSource
--------------------------------

In ouster-sdk 0.11.0 release, the slice ``[::]`` operator used to return a list of ``LidarScan`` objects (or list of
lists for the ``MultiScanSource`` case). However, starting with ouster-sdk 0.12.0 the ScanSource slice operator ``[::]``
returns a ``ScanSource`` scoped to the indicated slice range. This means that the users can pass the object returned by
the slice operator ``[::]`` to any function or code that expects a ``ScanSource`` object (or ``MultiScanSource``). The
following snippet shows few examples to demonstrate this capability:

.. code:: python

   >>> # ... continuing with the `source` object from the previous example
   >>> source2 = source[5:10]
   >>> print("source2 length:", len(source2))   # This should print 5 since source2 is scoped to the range [5, 10)
   >>> print(source2[0].frame_id)   # This is equivalent to calling `print(source[5].frame_id)`
   >>> print(source2[4].frame_id)   # This is equivalent to calling `print(source[9].frame_id)`
   >>> # invoking source2[10].frame_id would result in an `out of range exception`` since source2 is scoped to 5 frames
   >>> source_iter = iter(source2)  # Use `source2` as an iterator similar to the main `source`
   >>> for scan in source_iter:
   ...     print(scan.frame_id)
   >>> # it is possible to sub slice, meaning take the result of a previous slice operation and slice it
   >>> # Thus, the following statement is valid
   >>> source3 = source2[2:4] # this would yield the same frames as `source3 = source[7:9]`
   >>> print("source3 length:", len(source3))   # Should print 2


Using the client API
====================

The client API provides :py:class:`.PacketMultiSource` implementations, as well as access to methods for configuring a sensor or reading metadata.

As such, you can use it instead of the ``ScanSource`` API if you prefer to work with individual packets rather than lidar scans.

Reading packets from a pcap file
--------------------------------

Because our sample pcap file contains the UDP packet stream but not the sensor metadata, we load the sensor
information from ``metadata_path`` first, using the client module:

.. code:: python

   >>> from ouster.sdk import client
   >>> with open(metadata_path, 'r') as f:
   ...     info = client.SensorInfo(f.read())

Now that we've parsed the metadata file into a :py:class:`.SensorInfo`, we can use it to read our
captured UDP data by instantiating :py:class:`.pcap.PcapMultiPacketReader`. This class acts as a
:py:class:`.PacketMultiSource` and can be used in many of the same contexts as a real sensor.

.. code:: python

    >>> from ouster.sdk import pcap
    >>> source = pcap.PcapMultiPacketReader(pcap_path, metadatas=[info])
    >>> for sensor_idx, packet in source:
    >>>    ...

The ``source`` object returned is an ``Iterable`` of tuples each containing the sensor index and a packet that
originates from that sensor. (The index corresponds to the positions of the ``SensorInfo`` instances provided to the
``metadatas`` keyword argument.)

To limit the output to an ``Iterable`` of packets (without the sensor index) use the ``single_source`` method, providing
the index of the sensor you want to read from (which will be ``0`` if you've only provided a single ``SensorInfo``
instance.)

.. code:: python

    >>> for packet in source.single_source(0):
    >>>    ...

To visualize data from this pcap file, proceed to :doc:`/python/examples/visualizations` examples.

.. _Lidar Data Format: https://static.ouster.dev/sensor-docs/sw_manual/sensor_data/sensor-data.html#lidar-data
.. _IMU Data Format: https://static.ouster.dev/sensor-docs/sw_manual/sensor_data/sensor-data.html#imu-data
.. _OS2 bridge sample data: https://data.ouster.io/sdk-samples/OS2/OS2_128_bridge_sample.zip


Using an Ouster Sensor
======================

If you have access to sensor hardware, you can start reading data by
instantiating a :py:class:`.PacketSource` that listens for a UDP data stream on
a local socket.

.. note::

   Connecting to an Ouster sensor is covered in the `Networking Guide`_ section of the Ouster
   Sensor Documentation.

In the following, ``<SENSOR_HOSTNAME>`` should be substituted for the actual hostname or IP of your
sensor. 

To make sure everything is connected, open a separate console window and try pinging the sensor. You
should see some output like:

.. tabs::

    .. code-tab:: console Linux/macOS

       $ ping -c1 <SENSOR_HOSTNAME>
       PING <SENSOR_HOSTNAME> (192.0.2.42) 56(84) bytes of data.
       64 bytes from <SENSOR_HOSTNAME> (192.0.2.42): icmp_seq=1 ttl=64 time=0.217 ms

    .. code-tab:: powershell Windows x64

       PS > ping /n 10 <SENSOR_HOSTNAME>
       Pinging <SENSOR_HOSTNAME> (192.0.2.42) with 32 bytes of data:
       Reply from 192.0.2.42: bytes=32 time=101ms TTL=124


Next, you'll need to configure the sensor with the config parameters using the client module.

In your open python session, set ``hostname`` as ``<SENSOR_HOSTNAME>``:

.. code:: python

   >>> hostname = '<SENSOR_HOSTNAME>'

Now configure the client:

.. code:: python

   >>> from ouster.sdk import client
   >>> config = client.SensorConfig()
   >>> config.udp_profile_lidar = client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL
   >>> config.udp_port_lidar = 7502
   >>> config.udp_port_imu = 7503
   >>> config.operating_mode = client.OperatingMode.OPERATING_NORMAL
   >>> client.set_config(hostname, config, persist=True, udp_dest_auto = True)

When using a :py:class:`.SensorPacketSource`, provide a list of hostname/``SensorConfig`` pairs:

.. code:: python

   >>> source = client.SensorPacketSource([(hostname, config)])
   >>> info = source.metadata

Now we have a ``source`` from our sensor! As before, since ``SensorPacketSource`` supports multiple sensors, you have
the option to read it as an ``Iterable[List[Optional[LidarScan]]]`` of each containing the sensor index and a scan, or to limit the output
to a single sensor (an ``Iterable[LidarScan]``.)

.. code:: python

   >>> for scans in source:
   >>>    ...  # a list of scans, one per sensor

Or,

.. code:: python

   >>> for scan in source.single_source(0):
   >>>    ...  # a single scan for sensor index 0

At this point, you're ready visualize to visualize data from your sensor. Proceed to
:doc:`/python/examples/visualizations` for examples. Or you can check out other things you can do with a
``source`` in the Python :doc:`/python/examples/index`.


.. _Networking Guide: https://static.ouster.dev/sensor-docs/image_route1/image_route3/networking_guide/networking_guide.html


Next Steps
==========

Now that you know the basics, you can check out our annotated examples for a more detailed look at
how to work with our data.

Here are a few things you might be interested in:

    * :ref:`ex-basic-sensor`
    * :ref:`ex-packets`
    * :doc:`/reference/lidar-scan`
    * :ref:`ex-record-stream-viz`
