.. _quickstart:

==================================
Quick Start with Ouster Python SDK
==================================
This quickstart guide will walk you through visualizing Ouster sensor data quickly, whether from
sample data or a live sensor. You should already have :ref:`installed<installation-ref>` the Ouster Python SDK. 

You'll want to start an interactive Python session and keep it open through the sections, as we'll
be reusing variables created in earlier parts.  To get started, open python and import the ouster
client::
    
    from ouster import client

Using Sample Data
=================
Download some `sample data`_ (**1.6 GB**) and unzip the download. You should
have two files:

  * ``OS1_128.pcap``
  * ``OS1_2048x10_128.json``

.. note::

    You can also get the full sample data collection of different sensor
    models, configurations and environments by visiting `Ouster Sample Data`_ page.

In the following parts of this Quick Start guide we will refer to these files
as using variable names ``pcap_file`` and ``metadata_file``.

.. code:: python

   # Path to .pcap sample data files
   pcap_file = '/path/to/OS1_128.pcap'
   metadata_file = 'path/to/OS1_2048x10_128.json'

``pcap_file`` represents the `captured packets` from the network in `PCAP format`_.
In our case it's UDP packets with Imu (:py:class:`.ImuPacket`) and lidar data
(:py:class:`.LidarPacket`). You can read more about internal structure
of  `Lidar Data Format`_ and `Imu Data Format`_ in Ouster Software User Manual.

``metadata_file`` is the Ouster Lidar sensor state at the moment of data recording
which contains the sensor parameters, intrinsics calibration and configuration data
that is needed for correct interpretation of data packets from ``pcap_file``.

Getting packet stream from pcap file
------------------------------------

Because ``pcap_file`` contains the packets but no sensor metadata we are loading
it separately from ``metadata_file`` first:

.. code:: python

    def read_metadata(metadata_file: str) -> client.SensorInfo:
        with open(metadata_file, 'r') as f:
            return client.SensorInfo(f.read())

    metadata = read_metadata(metadata_file)

Now we are ready to create :py:class:`.pcap.Pcap` packet source and read packets:

.. code:: python

    from ouster import pcap
    from contextlib import closing

    with closing(pcap.Pcap(pcap_file, metadata)) as pcap_source:
        for packet in pcap_source:
            if isinstance(packet, client.LidarPacket):
                print('doing things with LidarPacket:')
                encoder_counts = packet.view(client.ColHeader.ENCODER_COUNT)
                timestamps = packet.view(client.ColHeader.TIMESTAMP)
                ranges = packet.view(client.ChanField.RANGE)
                print(f'  encoder counts = {encoder_counts.shape}')
                print(f'  timestamps = {timestamps.shape}')
                print(f'  ranges = {ranges.shape}')

            if isinstance(packet, client.ImuPacket):
                print('doing things with ImuPacket:')
                print(f'  acceleration = {packet.accel}')
                print(f'  angular_velocity = {packet.angular_vel}')

Notice how we use ``metadata`` which is the :py:class:`.SensorInfo` object
that we created early.

We use :py:func:`.closing()` context to ensure that :py:class:`.PacketSource`
and corresponging pcap file will be closed later. Keep an eye on importance to
close and free resources especially once you start reading data from live sensor.


.. _sample data: https://data.ouster.io/sdk-samples/OS1/OS1_128_sample.zip
.. _Lidar Data Format: https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf#10
.. _Imu Data Format: https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf#13
.. _PCAP format: https://tools.ietf.org/id/draft-gharris-opsawg-pcap-00.html
.. _Ouster Sample Data: https://ouster.com/resources/lidar-sample-data/


Using an Ouster Sensor
=======================
If you have a live sensor, we can now duplicate the same work from above making a :py:class:`.PacketSource`,
but using your sensor as the source. If you don't have an Ouster sensor (yet!), keep your Python
interpreter open and jump to the next section, `Visualizing your data in 3D`_.

.. note:: 
    Connecting to an Ouster sensor is covered in the Networking Guide section of ``Software User
    Manual`` available on `our website Downloads page`_.

For convenience, let's store the sensor hostname or IP as an enviornment variable::

    $ export SENSOR_HOSTNAME=<sensor hostname or assigned IP address>

To make sure everything is connected, try pinging the sensor::

    $ ping -c1 $SENSOR_HOSTNAME

Next, configure the sensor with the correct ``UDP_DEST``. In your python interpreter::

    hostname = '<SENSOR_HOSTNAME>'
    udp_dest = '<UDP_DEST>'
    config = client.SensorConfig()
    config.udp_dest = udp_dest

    client.set_config(hostname, config)

.. note::

   ``UDP_DEST`` is the destination to which the sensor sends UDP traffic. On
   boot, the sensor will not output data until this is set.

.. _our website Downloads page: https://ouster.com/downloads/

Just like with our sample data, we will want to create a :py:class:`.PacketSource`
from our sensor::
    
    source = client.Sensor(hostname)


Visualizing your data in 2D
=============================

Quick example on how to combine the stream of lidar packets that we've got
earlier into :py:class:`.LidarScan` object and visualize the range data as 2D
range image.

We will read from ``source`` to get the frame number with some interesing
view.

.. code:: python

    from more_itertools import nth

    with closing(pcap.Pcap(pcap_file, metadata)) as source:
        
        # start stream of LidarScan object and get 84th scan
        scan = nth(client.Scans(source), 84)

        # range measurements for full LidarScan
        range_field = scan.field(client.ChanField.RANGE)

        # shift rows of the fields according to sensor intrincics to
        # recover 2D image
        range_img = client.destagger(source.metadata, range_field)

        # Plot first 512 column of range data (our of 2048 for better view)
        plt.imshow(range_img[:, 0:512], cmap='gray', resample=False, vmax=50000)
        plt.axis('off')
        plt.show()

        
.. figure:: images/lidar_scan_range_image.png
    :align: center

    LidarScan ``RANGE`` field. Visualised only first 512 column out of 2048
    with simple gray color mapping.

For more in-depth explanation of API concepts and examples with the sample
data see :ref:`ex-api-concepts-sample`.

.. todo::

   - nice explanation of XYZ Look up table aka XYZLut
   - some nice explanation of ``client.Sensor`` ??


Visualizing your data in 3D
=============================
If you've been through either one of the previous sections (or both!), you should now have a
``source`` from which you can obtain sensor data::

    from contextlib import closing
    import matplotlib.pyplot as plt

    metadata = source.metadata

    with closing(client.Scans(source)) as scans:
        scan = next(iter(scans))

    xyzlut = client.XYZLut(metadata)
    xyz = xyzlut(scan)

    [x, y, z] = [c.flatten() for c in np.dsplit(xyz, 3)]
    ax.scatter(x, y, z, c=z / max(z), s=0.2)
    plt.show()

If you want to learn more about how we transformed the ``scan`` into 3D coordinates to graph, see
:ref:`ex-xyzlut`.

What Next
=========

You have now officially visualized Ouster lidar in both 2D and 3D data using the Ouster
Python SDK! Now that you know the basics, you can check out our annotated examples for
a more detailed look at how to work with our data.

Here are a few things you might be interested in:

    * :ref:`ex-api-concepts-sample`
    * :ref:`ex-metadata`
    * :ref:`ex-xyzlut`
    * :ref:`ex-streaming-and-destaggering`
    * :ref:`ex-imu`
    
.. todo::
    - Api docs link
    - Github Ouster SDK <https://github.com/ouster-lidar/ouster_example>

