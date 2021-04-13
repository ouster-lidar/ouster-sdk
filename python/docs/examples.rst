==================
Annotated Examples
==================

.. _ex-metadata:

Obtaining metadata
-------------------
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
--------------------------------------
.. literalinclude:: /../src/ouster/sdk/examples.py
    :start-after: [doc-stag-plot-xyz-points]
    :end-before: [doc-etag-plot-xyz-points]
    :emphasize-lines: 2-5
    :linenos:


In order to translate a ``LidarScan`` into 3D, we create the function ``xyzlut``, which projects any
``LidarScan`` into cartesian coordintes. We then apply it to ``scan`` to create ``xyz``, a numpy
array of ``H x W x 3`` which gives the cartesian coordinates of the scan in the sensor coordinate
frame.

To run the code in this example, try::

    $ python -m ouster.sdk.examples $SENSOR_HOSTNAME plot-xyz-points

That should open a 3D plot of a single scan of your location taken just now by your sensor. You
should be able to recognize the contours of the scene around you.


.. _ex-streaming-and-destaggering:

Streaming, 2D Representations, and Destaggering
-----------------------------------------------

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

Notice that instead of taking a ``sample`` as we did above with ``plot-xyz-points``, we used
``stream``, which allows for a continuous live data stream. We close the ``stream`` when we are
finished, hence the use of ``closing`` in the first highlighted line. 

In the second set of highlighted lines, we ``client.destagger`` to correctly arrange the points into
a 2D view which makes visual sense. Otherwise every column would represent the points captured at
the same timestamp instead of the same azimuth angle.

To exit the visualization, you can use ``ESC``.

.. _ex-imu:

Working with IMU data from the Ouster sensor
--------------------------------------------
IMU data from the Ouster sensor can be read as ``client.ImuPacket``. Like other ``Packets``, you can
get them from a ``PacketSource``::
    
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

