.. _quickstart:

==================================================
Developer's Quick Start with the Ouster Python SDK
==================================================

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

Because our pcap file contains the UDP packet stream but not the sensor metadata, we load the sensor
information from ``metadata_path`` first, using the client module:

.. code:: python

   >>> from ouster import client
   >>> with open(metadata_path, 'r') as f:
   ...     info = client.SensorInfo(f.read())

Now that we've parsed the metadata file into a :py:class:`.SensorInfo`, we can use it to read our
captured UDP data by instantiating :py:class:`.pcap.Pcap`. This class acts as a
:py:class:`.PacketSource` and can be used in many of the same contexts as a real sensor.

.. code:: python

    >>> from ouster import pcap
    >>> source = pcap.Pcap(pcap_path, info)

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

   >>> from ouster import client
   >>> config = client.SensorConfig()
   >>> config.udp_port_lidar = 7502
   >>> config.udp_port_imu = 7503
   >>> config.operating_mode = client.OperatingMode.OPERATING_NORMAL
   >>> client.set_config(hostname, config, persist=True, udp_dest_auto = True)

Just like with the sample data, you can create a :py:class:`.PacketSource` from the sensor:

.. code:: python

   >>> source = client.Sensor(hostname)
   >>> info = source.metadata

Now we have a ``source`` from our sensor! You're ready to record, visualize to visualize data from your sensor, proceed to
:doc:`/python/examples/visualizations` examples. Or you can check out other things you can do with a
``source`` in the Python :doc:`/python/examples/index`.


.. _Networking Guide: https://static.ouster.dev/sensor-docs/developer_common_sections/networking-guide.html


Next Steps
==========

Now that you know the basics, you can check out our annotated examples for a more detailed look at
how to work with our data.

Here are a few things you might be interested in:

    * :ref:`ex-basic-sensor`
    * :ref:`ex-packets`
    * :doc:`/reference/lidar-scan`
    * :ref:`ex-record-stream-viz`
