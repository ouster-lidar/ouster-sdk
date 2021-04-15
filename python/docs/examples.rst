==============================
Annotated Examples & Concepts
==============================

Below is a loosely connected collection of examples and concepts based on
using Ouster SDK with samples (pcap) files and with live sensor.

.. _ex-api-concepts-sample:

API Concepts with sample data
==============================

In this part we continue building on top of :ref:`Quick Start <quickstart>`
sample data parts and we assume that ``pcap_file`` and ``metadata_file``
already present as variables:

.. code:: python

   # Path to .pcap sample data files
   pcap_file = '/path/to/OS1_128.pcap'
   metadata_file = 'path/to/OS1_2048x10_128.json'


Sensor metadata as :py:class:`.SensorInfo`
------------------------------------------

The content of ``metadata_file`` (json) is represented by an object
:py:class:`.SensorInfo`. It's the main object that represents the
sensor state at the particular moment in time and holds information about
configured parameters, lidar data mode and intrinsics calibration which are
different for every sensor.

Thus we almost always need a :py:class:`.SensorInfo` object to
make sense of any particular stream of Ouster sensor packets data.

Because ``pcap_file`` internally doesn't store the sensor metadata we need first
to read it from a separate json file (``metadata_file``). We can do it like this:

.. code:: python

    def read_metadata(metadata_file: str) -> client.SensorInfo:
        with open(metadata_file, 'r') as f:
            return client.SensorInfo(f.read())

    metadata = read_metadata(metadata_file)

.. note::

    Always use the correct ``metadata_file`` with recorded sensor data streams!
 
    It's tempting to reuse the same ``metadata_file`` with multiple recorded
    pcap files but it's a sure way to get yourself into the trouble. The thing
    is that every sensor has different intrinsics (``beam_azimuth_angles``,
    ``beam_altitude_angles``), different internal ``data_format`` parameters that
    defines things like ``columns_per_frame``, ``columns_per_packet``,
    ``pixel_shift_by_row``. Also for specific recorded session the configuration of
    ``lidar_mode``, ``column_window`` and other parameters might differ which
    is all influence to the quality of point cloud that is reconstructed from
    wire format UDP packets.
 
    So, please, always be careful which ``metadata_file`` you use to read
    ``pcap_file`` and always record the metadata along with recording data from
    a sensor.
 

Sensor data as a :py:class:`.PacketSource`
------------------------------------------

:py:class:`.PacketSource` is the basic interface to get the sensor data as
a stream of packets (:py:class:`.ImuPacket` or :py:class:`.LidarPacket`).

To get a fully configured and ready to use :py:class:`.PacketSource` from
``pcap_file`` we use :py:class:`.pcap.Pcap`:


.. code:: python

    pcap_source = pcap.Pcap(pcap_file, metadata)


.. note::

    Here we are re-using ``metadata`` object that we've got from ``metadata_file``
    earlier.

Now we can read packets from ``pcap_source`` (which conforms to
:py:class:`.PacketSource`) with the following code:

.. code:: python

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

:py:class:`.PacketSource` objects also provides access to the sensor metadata via
:py:attr:`.PacketSource.metadata` attribute.

Later we will see how to get :py:class:`.PacketSource` from the live Ouster sensor
but first let's look how we can construct the full :py:class:`.LidarScan` from the
stream of individual :py:class:`.LidarPacket`'s


Single frame of lidar data as a :py:class:`.LidarScan`
-------------------------------------------------------

Every :py:class:`.LidarPacket` contains a set of measurements that comprise a
small part of the full 360 rotation of the lidar.

.. note::

   For example OS1-128 in ``1024x10`` mode contains 16 Measurement Blocks per one
   :py:class:`.LidarPacket` and thus needs 64 lidar packets of data for a full
   :py:class:`.LidarScan`.

   For more details please refer to `Lidar Data Format`_ in Ouster Software
   User Manual.

To get a stream of :py:class:`.LidarScan`'s we use the :py:class:`.client.Scans`
object that transforms any :py:class:`.PacketSource` with lidar packets to
scans:

.. code:: python

    for scan in client.Scans(pcap_source):
        intensities = scan.field(client.ChanField.INTENSITY)
        ranges = scan.field(client.ChanField.RANGE)
        print(f'intensities = {intensities.shape}')
        print(f'ranges = {ranges.shape}')


We can visualize range measurements (``ranges``) from LidarScan since it's
a ``np.ndarray`` of size ``H x W`` (``128 x 2048`` for our particular example).
Let's see one in detail:

.. figure:: images/lidar_scan_staggered.png
   :align: center

   LidarScan ``RANGE`` field visualised with :py:func:`matplotlib.pyplot.imshow()`
   and simple gray color mapping for better look.

Oh, that doesn't look like a normal image. Believe me it's a bicycle on
the left patch and trees with a parked cars on the right.

It's a good place to look into the field destaggering process next (and fix
the above image view)


Staggered vs Destaggered :py:class:`.LidarScan` field
-----------------------------------------------------

Default **staggered** representation of :py:class:`.LidarScan` is that one column
of a channel field view (:py:class:`.ChanField` selector) corresponds to one
Measurement Block of lidar packet which consists of returned values per every
pixel in a column and additional headers (see :py:class:`.ColHeader`).

For more natural 2D image field view representation that looks normal for humans
we need to **destagger** the field array first with :py:func:`.client.destagger`
function.

Below we get one lidar scan of a particular number from ``pcap_source`` and
plot destaggered range image.

.. code::

    import matplotlib.pyplot as plt
    from more_itertools import nth

    # ... see above examples for how to get `pcap_source` ...

    scans = client.Scans(pcap_source)

    # iterate `scans` and get the 84th LidarScan
    scan = nth(scans, 84)
    ranges = scan.field(client.ChanField.RANGE)

    # destagger ranges, notice `metadata` use, that is needed to get
    # sensor intrinsics and correctly data transforms
    ranges_destaggered = client.destagger(pcap_source.metadata, ranges)

    plt.imshow(ranges_destaggered, cmap='gray', resample=False)


.. figure:: images/lidar_scan_destaggered.png
    :align: center

    LidarScan ``RANGE`` field **destaggered** and visualized.

Now we can easily use this image in common visual tasks pipeline.

.. note::

    We keep **staggered** representation inside because it's convenient and
    computationaly faster to transform full LidarScan to 3D point cloud, correct for
    sensor movement in SLAM tasks or parallelize computation per column batches.

    BTW, you can view this particular scene in `Web Slam`_ to see reconstructed
    point cloud, rotate between different fields in **destaggered** image view
    and see the video of registered point clouds with our internal SLAM algorithm.

.. _Web Slam: https://webslam.ouster.dev/slam/1610482355.9361048.rVdW_dgws/


.. _ex-metadata:

Obtaining metadata from the live sensor
=======================================
Ouster sensors require metadata to interpret the readings of the sensor. Let's try dumping your
sensor's metadata::

    $ python -m ouster.sdk.examples $SENSOR_HOSTNAME get-metadata

That should have dumped a ``json`` object with fields such as ``lidar_mode`` and
``beam_azimuth_angles`` to your terminal.

Let's look inside the example we just ran:

.. literalinclude:: /../src/ouster/sdk/examples.py
    :start-after: [doc-stag-get-metadata]
    :end-before: [doc-etag-get-metadata]

When you work with a live sensor, you will not need to store this metadata as the client will
automatically fetch it, but recorded ``pcaps`` must always be accompanied by a ``json`` file
containing this object.

.. _ex-xyzlut:

Working with 3D Points, and the xyzlut
======================================
.. literalinclude:: /../src/ouster/sdk/examples.py
    :start-after: [doc-stag-plot-xyz-points]
    :end-before: [doc-etag-plot-xyz-points]
    :emphasize-lines: 2-5
    :linenos:

In order to translate a :py:class:`.LidarScan` into 3D, we create the function
``xyzlut`` (via :py:func:`.client.XYZLut`), which projects any :py:class:`.LidarScan`
into cartesian coordintes. We then apply it to ``scan`` to create ``xyz``, a
numpy array of ``H x W x 3`` which gives the cartesian coordinates of the scan in
the sensor coordinate frame.

To run the code in this example, try::

    $ python -m ouster.sdk.examples $SENSOR_HOSTNAME plot-xyz-points

That should open a 3D plot of a single scan of your location taken just now by your sensor. You
should be able to recognize the contours of the scene around you.


.. _ex-streaming-and-destaggering:

Streaming, 2D Representations, and Destaggering
===============================================

Sometimes it is preferable to work with the 2D representation of the data, for example when working
with the intensity::
    
    $ python -m ouster.sdk.examples $SENSOR_HOSTNAME live-plot-intensity

This should give you a live feed from your sensor that looks like a black and white moving image.
Try waving your hand or moving around to find yourself within the image!

So how did we do that?

.. literalinclude:: /../src/ouster/sdk/examples.py
   :start-after: [doc-stag-live-plot-intensity]
   :end-before: [doc-etag-live-plot-intensity]
   :emphasize-lines: 2, 9-10
   :linenos:

Notice that instead of taking a ``sample`` as we did above with ``plot-xyz-points``,
we used :py:meth:`.Scans.stream`, which allows for a continuous live data stream.
We close the ``stream`` when we are finished, hence the use of :py:func:`.closing` in
the first highlighted line.

In the second set of highlighted lines, we :py:func:`.client.destagger` to correctly
arrange the points into a 2D view which makes visual sense. Otherwise every column
would represent the points captured at the same timestamp instead of the same azimuth
angle.

To exit the visualization, you can use ``ESC``.

.. _ex-imu:

Working with IMU data from the Ouster sensor
============================================
IMU data from the Ouster sensor can be read as :py:class:`~.client.ImuPacket`.
Like other ``Packets``, you can get them from a :py:class:`.PacketSource`::
    
    with closing(source):
        imu_packet_list = [ p in time_limited(10, source) if isinstance(p, client.ImuPacket) ] 
        
You might decide, for example, that you wish to graph the acceleration in the z direction over time::
    
    ts, z_accel = zip(*[(p.sys_ts, p.accel[2]) for p in imu_packet_list)

Now you've created ``ts`` and ``z_accel`` which can then be graphed::
    
    fig, ax = plt.subplots(figsize=(12.0,2))
    ax.plot(ts, z_accel)

That should give you a graph, albeit without labeled axes. If you have a live sensor, and you want
to see the prettified graph, try the ``plot-imu-z-accel`` example::

    $ python -m ouster.sdk.examples $SENSOR_HOSTNAME plot-imu-z-accel

