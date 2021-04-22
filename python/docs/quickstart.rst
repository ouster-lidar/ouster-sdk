.. _quickstart:

==================================
Quick Start with Ouster Python SDK
==================================

This quickstart guide will walk you through visualizing Ouster sensor data quickly, whether from
sample data or a live sensor. The first step is to install the SDK with the optional dependencies
needed to run the examples::

  $ python3 -m pip install ouster-sdk[examples]

.. note::

   To run the example code on Windows 10, you may also find that you need the ``PyQt5`` library.

.. todo::

   Add PyQt5 as a dependency to the ``examples`` extra on Windows

.. todo::

   Add venv example use in example above. I was about to add those ``python -m venv VENV && source ...`` lines but not sure how to nicely put it for Windows users too.

You'll want to start an interactive Python session and keep it open through the sections, as we'll
be reusing variables created in earlier parts.  To get started, open python and import the ouster
client:

.. code:: python
    
   >>> from ouster import client

If you'd like to start by working with sample data, continue to the section below. If you'd prefer
to start capturing data from a sensor, you can skip to `Using an Ouster Sensor`_ below.


Using Sample Data
=================

Download the `sample data`_ (**1.6 GB**) and unzip the contents. You should have two files:

  * ``OS1_128.pcap``
  * ``OS1_2048x10_128.json``

The downloaded pcap file contains lidar and imu packets captured from the network . You can read
more about the `IMU Data Format`_ and `Lidar Data Format`_ in the Ouster Software User Manual. The
JSON file contains metadata queried from the sensor TCP interface necessary to interpret the packet
data.

Let's load the paths into your open session of python:

.. code:: python

   >>> pcap_path = '/path/to/OS1_128.pcap'
   >>> metadata_path = '/path/to/OS1_2048x10_128.json'

.. note::

    The full sample data collection, spanning various sensor models, beam configurations,
    environments, and use cases, is available at the `Ouster Sample Data`_ page.

Because our pcap file contains the UDP packet stream but not the sensor metadata, we load the
metadata separately from ``metadata_path`` first:

.. code:: python
 
   >>> with open(metadata_path, 'r') as f:
   ...     metadata = client.SensorInfo(f.read())

Now that we've parsed the metadata file into a :py:class:`.SensorInfo`, we can use it to read our
captured UDP data by instantiating :py:class:`.pcap.Pcap`. This class acts as a
:py:class:`.PacketSource` and can be used in many of the same contexts as a real sensor.

.. code:: python

    >>> from ouster import pcap
    >>> source = pcap.Pcap(pcap_path, metadata)

To visualize data from this pcap file, proceed to `Visualizing Lidar Data`_ below.


.. _sample data: https://data.ouster.io/sdk-samples/OS1/OS1_128_sample.zip
.. _Lidar Data Format: https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf#10
.. _IMU Data Format: https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf#13
.. _Ouster Sample Data: https://ouster.com/resources/lidar-sample-data/


Using an Ouster Sensor
======================

If you have access to sensor hardware, you can start reading data by instantiating a
:py:class:`.PacketSource` that listens for a UDP data stream on a local socket.

.. note::

   Connecting to an Ouster sensor is covered in the `Networking Guide`_ section of the Ouster
   Software User Manual.

In the following, ``<SENSOR_HOSTNAME>`` shold be substituted for the actual hostname or IP of your
sensor and ``<UDP_DEST>`` should be the hostname or IP of the machine reading sensor data, per the
network configuration.

To make sure everything is connected, try pinging the sensor. You should see some output like::

   $ ping -c1 <SENSOR_HOSTNAME>
   PING <SENSOR_HOSTNAME> (192.0.2.42) 56(84) bytes of data.
   64 bytes from <SENSOR_HOSTNAME> (192.0.2.42): icmp_seq=1 ttl=64 time=0.217 ms

Next, you'll need to configure the sensor with the correct destination address or IP

.. code:: python

   >>> hostname = '<SENSOR_HOSTNAME>'
   >>> config = client.SensorConfig()
   >>> config.udp_dest = '<UDP_DEST>'
   >>> config.udp_port_lidar = 7502
   >>> config.udp_port_imu = 7503
   >>> config.operating_mode = client.OperatingMode.OPERATING_NORMAL
   >>> client.set_config(hostname, config)

Just like with the sample data, you can create a :py:class:`.PacketSource` from the sensor:
    
.. code:: python

   >>> source = client.Sensor(hostname, _overflow_err=False)


.. _Networking Guide: https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf#64


Visualizing Lidar Data
======================

At this point, you should have defined ``source`` using either a pcap file or UDP data streaming
directly from a sensor. Let's read from ``source`` until we get to the 84th frame of data:

.. code:: python

   >>> from contextlib import closing
   >>> from more_itertools import nth
   >>> with closing(client.Scans(source)) as scans:
   ...     scan = nth(client.Scans(source), 84)
   >>> scan
   <ouster.client.data.LidarScan object at 0x7f7ccc35fba8>

Now that we have a frame of data available as a `py:class:.LidarScan` datatype, we can extract the
range measurments and turn them into a range image, where each column corresponds to a single
azimuth angle:

.. code:: python

   >>> range_field = scan.field(client.ChanField.RANGE)
   >>> range_img = client.destagger(source.metadata, range_field)

We can plot the results using standard Python tools that work with numpy datatypes. Here, we extract
the first 512 columns of range data and display the result:

.. code:: python

   >>> import matplotlib.pyplot as plt
   >>> plt.imshow(range_img[:, 0:512], cmap='gray', resample=False)
   >>> plt.axis('off')
   >>> plt.show()

For a more in-depth explanation of the API concepts involved with visualizing your data in 2D see
:ref:`ex-staggered-and-destaggered`.

.. figure:: images/lidar_scan_range_image.png
   :align: center

   LidarScan ``RANGE`` field. Visualizing only the first 512 column out of 2048 with simple gray
   color mapping.

We can also plot the results in 3D by projecting the range measurements into cartesian
coordinates. To do this, we first create a lookup table, then use it to produce X, Y, Z coordinates
from our scan data:

.. code:: python

    >>> xyzlut = client.XYZLut(metadata)
    >>> xyz = xyzlut(scan)

Lastly, we need to re-arrange the resulting numpy array into a shape that's suitable for plotting:

    >>> import numpy as np
    >>> [x, y, z] = [c.flatten() for c in np.dsplit(xyz, 3)]
    >>> ax = plt.axes(projection='3d')
    >>> r = 30
    >>> ax.set_xlim3d([-r, r])
    >>> ax.set_ylim3d([-r, r])
    >>> ax.set_zlim3d([0, 2 * r])
    >>> ax.scatter(x, y, z, c=z / max(z), s=0.2)
    >>> plt.show()

If you want to learn more about how we transformed the ``scan`` into 3D coordinates to graph, see
:ref:`ex-xyzlut`.

.. figure:: images/lidar_scan_xyz.png
   :align: center

   Point cloud from sample data. Points colored by Z coordinate value.


Next Steps
==========

You have now officially visualized Ouster lidar in both 2D and 3D data using the Ouster Python SDK!
Now that you know the basics, you can check out our annotated examples for a more detailed look at
how to work with our data.

Here are a few things you might be interested in:

    * :ref:`ex-metadata`
    * :ref:`ex-packets`
    * :ref:`ex-lidar-scans`
    * :ref:`ex-staggered-and-destaggered`
    * :ref:`ex-xyzlut`
    * :ref:`ex-streaming`
    * :ref:`ex-pcap-record`
    * :ref:`ex-pcap-live-preview`
    * :ref:`ex-imu`
    
.. todo::
    - Api docs link
    - Github Ouster SDK <https://github.com/ouster-lidar/ouster_example>

