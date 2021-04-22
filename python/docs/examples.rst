===================
Examples & Concepts
===================

A loosely connected collection of examples and concepts useful for working with Ouster SDK. If you
are just starting, you may wish to start with :ref:`quickstart`.

For convenience, throughout the examples and concepts we will use ``pcap_path`` and
``metadata_path`` to refer to the variables from the Quick Start Guide.


.. _ex-metadata:


Sensor Metadata and :py:class:`.SensorInfo`
===========================================

Ouster sensors require metadata to interpret the readings of the sensor. Represented by the object
:py:class:`.SensorInfo`, its fields include configuration parameters such as 
``lidar_mode`` and and sensor intrinsics like ``beam_azimuth_angles``.

When you work with a live sensor, the client will automatically fetch the metadata, but recorded
``pcaps`` must always be accompanied by a ``json`` file containing this object.

.. note::

    Always use the correct ``metadata_path`` with recorded sensor data streams!

    Using a metadatafile from another sensor or from a previous run with the wrong configuration
    parameters will error out or give you live-than-desired precision.

Since it's crucial to save the correct metadata file, let's see how we can get that from a live
sensor. Try running the following example::

    $ python -m ouster.sdk.examples.client $SENSOR_HOSTNAME get-metadata

And now let's look inside the example we just ran:

.. literalinclude:: /../src/ouster/sdk/examples/client.py
    :start-after: [doc-stag-get-metadata]
    :end-before: [doc-etag-get-metadata]
    :dedent:

Seems simple enough!

.. _ex-packets:


Working Directly with Packets via a :py:class:`.PacketSource`
=============================================================

The :py:class:`.PacketSource` is the basic interface to get the sensor data as a stream of packets
(:py:class:`.ImuPacket` or :py:class:`.LidarPacket`).

To get a fully configured and ready to use :py:class:`.PacketSource` from ``pcap_path`` we use
:py:class:`.pcap.Pcap`:

.. code:: python

    metadata = client.SensorInfo(metadata_path)
    source = pcap.Pcap(pcap_path, metadata)

Now we can read packets from ``source`` (which conforms to :py:class:`.PacketSource`) with the
following code:

.. literalinclude:: /../src/ouster/sdk/examples/pcap.py
    :start-after: [doc-stag-pcap-read-packets]
    :end-before: [doc-etag-pcap-read-packets]
    :dedent:

Once created, :py:class:`.PacketSource` objects provide access to the sensor metadata via the
:py:attr:`.PacketSource.metadata` attribute.


.. _ex-lidar-scans:


Working with Frames of Lidar Data as a :py:class:`.LidarScan`
=============================================================

It is often convenient to work with the full 360-degree rotations of the lidar we know as frames,
instead of working with individual packets. The :py:class:`.LidarPacket` contains a full frame of
lidar measurements.

To get a stream of :py:class:`.LidarScan`'s we use the :py:class:`.client.Scans` object that
transforms any :py:class:`.PacketSource` with lidar packets to scans:

.. code:: python

    for scan in client.Scans(source):
        intensities = scan.field(client.ChanField.INTENSITY)
        ranges = scan.field(client.ChanField.RANGE) 
        print(f'intensities = {intensities.shape}')
        print(f'ranges = {ranges.shape}')


.. _ex-staggered-and-destaggered:


Staggered vs Destaggered 2D Representations
===========================================

The default **staggered** representation of :py:class:`.LidarScan` contains columns which pertain to
measurements taken at a single timestamp, corresponding directly with the data as it comes in. For a
more natural 2D image, we therefore **destagger** the field with the :py:func:`.client.destagger`
function.

Let's take a look at a typical **staggered** representation:

.. figure:: images/lidar_scan_staggered.png
   :align: center

   LidarScan ``RANGE`` field visualized with :py:func:`matplotlib.pyplot.imshow()` and simple gray
   color mapping for better look.

Oh, that doesn't look like a normal image! What could it be? Surely not a
bicycle in the left patch, and trees with a parked car on the right...

Let's see if we can destagger the image and decipher the scene:

.. code::

    import matplotlib.pyplot as plt
    from more_itertools import nth

    metadata = client.SensorInfo(metadata_path)
    source = pcap.Pcap(pcap_path, metadata)

    scans = client.Scans(source)

    # iterate `scans` and get the 84th LidarScan
    scan = nth(scans, 84)
    ranges = scan.field(client.ChanField.RANGE)

    # destagger ranges, notice `metadata` use, that is needed to get
    # sensor intrinsics and correctly data transforms
    ranges_destaggered = client.destagger(source.metadata, ranges)

    plt.imshow(ranges_destaggered, cmap='gray', resample=False)

This should give the scene below, which we have blown up sections of for better visiblity.

.. figure:: images/lidar_scan_destaggered.png
    :align: center

    **destaggered** LidarScan ``RANGE`` field

There's our man on a bicycle, and our beautiful car and tree foregrounded
against farther-away trees! Now that this makes visual sense, we can easily use
our data in common visual task pipelines.

.. note::

    By the way, you can view this particular scene in both 2D and 3D at Ouster's `Web Slam`_! Use
    your mouse to click and move the 3D scene, and the listed controls to rotate between different
    destaggered image views. The video at the bottom shows the registered point clouds of our
    internal SLAM algorithm. 

.. _Web Slam: https://webslam.ouster.dev/slam/1610482355.9361048.rVdW_dgws/


.. _ex-xyzlut:


Working with 3D Points and the XYZLut
=====================================

To facilitate working with 3D points, you can create a function via :py:func:`.client.XYZLut` which
will project any :py:class:`.LidarScan` into cartesian coordinates by referencing a precalculated XYZ
Look-uptable. This function can then be applied to any scan to create a numpy array of ``H x W x
3``, which represents the cartesian coordintes of the points in the sensor coordinate frame.

.. literalinclude:: /../src/ouster/sdk/examples/client.py
    :start-after: [doc-stag-plot-xyz-points]
    :end-before: [doc-etag-plot-xyz-points]
    :emphasize-lines: 2-5
    :linenos:
    :dedent:

If you have a live sensor, you can run this code with one of our examples::

    $ python -m ouster.sdk.examples.client $SENSOR_HOSTNAME plot-xyz-points

That should open a 3D plot of a single scan of your location taken just now by your sensor. You
should be able to recognize the contours of the scene around you.

If you don’t have a live sensor, you can run this code with our pcap examples::

    $ python -m ouster.sdk.examples.pcap OS1_128.pcap \
      --info-json OS1_2048x10_128.json plot-xyz-points --scan-num 84


.. figure:: images/lidar_scan_xyz_84.png
   :align: center

   Point cloud from sample data (scan 84). Points colored by ``INTENSITY`` value.

For details check the source code of an example :func:`.examples.pcap.pcap_display_xyz_points`

.. _ex-streaming:


Streaming Live Data
===================

Instead of working with a recorded dataset or a few captured frames of data, let's see if we can get
a live feed from the senseor::
    
    $ python -m ouster.sdk.examples.client $SENSOR_HOSTNAME live-plot-intensity

This should give you a live feed from your sensor that looks like a black and white moving image.
Try waving your hand or moving around to find yourself within the image!

So how did we do that?

.. literalinclude:: /../src/ouster/sdk/examples/client.py
   :start-after: [doc-stag-live-plot-intensity]
   :end-before: [doc-etag-live-plot-intensity]
   :emphasize-lines: 2-3
   :linenos:
   :dedent:

Notice that instead of taking a ``sample`` as we did in previous example, we used
:py:meth:`.Scans.stream`, which allows for a continuous live data stream.  We close the ``stream``
when we are finished, hence the use of :py:func:`.closing` in the highlighted line.

To exit the visualization, you can use ``ESC``.


.. _ex-pcap-record:


Recording Sensor Data to pcap
===============================

It's easy to record data to a pcap file from a live sensor programatically, let's try it first
with the following example::

    $ python -m ouster.sdk.examples.client $SENSOR_HOSTNAME record-pcap

This will capture the :class:`.client.LidarPacket`'s and :class:`.client.ImuPacket`'s data for *100*
seconds and store the pcap file along with the sensor info metadata json file into the current
directory.

The source code of an example below:

.. literalinclude:: /../src/ouster/sdk/examples/client.py
   :start-after: [doc-stag-pcap-record]
   :end-before: [doc-etag-pcap-record]
   :emphasize-lines: 25-30
   :linenos:
   :dedent:

Good! The resulting pcap and json files can be used with any examples in the :mod:`.examples.pcap`
module.


.. _ex-pcap-live-preview:


Pcap Live Data Preview
=======================

We can easily view the data that was recorded in the previous section. Based on an example from
:ref:`ex-streaming` above we have a pcap viewer with additional *stagger*/*destagger* handler on key
`D` and pause on `SPACE` key  (source code: :func:`.examples.pcap.pcap_2d_viewer`). To run it try
the following command::

    $ python -m ouster.sdk.examples.pcap OS1_128.pcap \
      --info-json OS1_2048x10_128.json 2d-viewer

Or substitute example data with pcap and json that you just recorded.


.. _ex-imu:


Working with IMU data from the Ouster sensor
============================================
IMU data from the Ouster sensor can be read as :py:class:`~.client.ImuPacket`.  Like other
``Packets``, you can get them from a :py:class:`.PacketSource`::
    
    with closing(source):
        imu_packet_list = [ p in time_limited(10, source) if isinstance(p, client.ImuPacket) ] 
        
You might decide, for example, that you wish to graph the acceleration in the z direction over time::
    
    ts, z_accel = zip(*[(p.sys_ts, p.accel[2]) for p in imu_packet_list)

Now you've created ``ts`` and ``z_accel`` which can then be graphed::
    
    fig, ax = plt.subplots(figsize=(12.0,2))
    ax.plot(ts, z_accel)

That should give you a graph, albeit without labeled axes. If you have a live sensor, and you want
to see the prettified graph, try the ``plot-imu-z-accel`` example::

    $ python -m ouster.sdk.examples.client $SENSOR_HOSTNAME plot-imu-z-accel

