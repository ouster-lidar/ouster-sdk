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
-----------------
Download some `sample data`_ and unzip the download. You should
have two files:

  * ``OS1_128.pcap``
  * ``OS1_2048x10_128.json``

  .. todo::
    - pcap-->PacketSource example
    - replace names of pcap/json
    - replace link to sample data
    - some nice explanation of ``client.Sensor`` ??

.. _sample data: https://link/to/sample/data 


Using an Ouster Sensor
----------------------
If you have a live sensor, we can now duplicate the same work from above making a ``PacketSource``,
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

    hostname = <SENSOR_HOSTNAME>
    udp_dest = <UDP_DEST>
    config = client.SensorConfig()
    config.udp_dest = udp_dest

    client.set_config(hostname, config)

.. _our website Downloads page: https://ouster.com/downloads/

Just like with our sample data, we will want to create a ``PacketSource`` from our sensor::
    
    source = client.Sensor(hostname)
    

Visualizing your data in 3D
---------------------------
If you've been through either one of the previous sections (or both!), you should now have a
``source`` from which you can obtain sensor data::

    from contextlib import closing
    import matplotlib.pyplot as plt

    metadata = source.metadata

    with closing(client.Scans(source)) as scans:
        scan = next(scans.__iter__())

    xyzlut = client.XYZLut(metadata)
    xyz = xyzlut(scan)

    [x, y, z] = [c.flatten() for c in np.dsplit(xyz, 3)]
    ax.scatter(x, y, z, c=z / max(z), s=0.2)
    plt.show()

If you want to learn more about how we transformed the ``scan`` into 3D coordinates to graph, see
:ref:`ex-xyzlut`.

What Next
=========

You have now officially visualized Ouster lidar data using the Ouster Python SDK! Now that you know
the basics, you can check out our annotated examples for a more detailed look at how to work with
our data.

Here are a few things you might be interested in:

    * :ref:`ex-metadata`
    * :ref:`ex-xyzlut`
    * :ref:`ex-streaming-and-destaggering`
    * :ref:`ex-imu`
    
.. todo::
    - Api docs link
    - Github Ouster SDK <https://github.com/ouster-lidar/ouster_example>

