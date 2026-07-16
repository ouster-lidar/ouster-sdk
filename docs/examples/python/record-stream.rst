Recording, Streaming, and Conversion
=====================================


To work with your sensor, you should configure the ports. See :doc:`set-configuration </features/sensor_config/using-the-api>` for more details.

Each config parameter corresponds directly to the sensor configuration parameters available on the
sensor.

You can run the above code, captured in the :py:func:`~.core.configure_sensor_params` example, as
follows:

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. code::

        $ python3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME configure-sensor

   .. tab-item:: Windows x64
      :sync: powershell

      .. code::

        PS > py -3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME configure-sensor

Once you've configured your sensor, you shouldn't have to configure it again until it shuts down or
restarts.  You can explore the ``persist`` flag to persist ``port`` and ``udp_dest`` settings over
sensor restarts.

Recording Sensor Data
---------------------

It's easy to record data to a pcap file from a sensor programmatically. Let's try it on a
:ref:`configured<ex-configure-sensor>` sensor:

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. code::

        $ python3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME record-pcap

   .. tab-item:: Windows x64
      :sync: powershell

      .. code::

        PS >  py -3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME record-pcap


This will capture the :class:`.core.LidarPacket`'s and :class:`.core.ImuPacket`'s data for 10
seconds and store the pcap file along with the metadata json file into the current directory.

.. note:: 

    TODO: Link to pcap write to file in MR 2

Good! The resulting pcap and json files can be used with any examples in the :mod:`.examples.pcap`
module.

.. _ex-stream:

Streaming Live Data
-------------------

Instead of working with a recorded dataset or a few captured frames of data, let's see if we can get
a live feed from your :ref:`configured<ex-configure-sensor>` sensor:

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. code::

        $ python3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME live-plot-reflectivity

   .. tab-item:: Windows x64
      :sync: powershell

      .. code::

        PS > py -3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME live-plot-reflectivity

This should give you a live feed from your sensor that looks like a black and white moving image.
Try waving your hand or moving around to find yourself within the image!

So how did we do that?

.. literalinclude:: /../python/src/ouster/sdk/examples/core.py
   :start-after: [doc-stag-live-plot-reflectivity]
   :end-before: [doc-etag-live-plot-reflectivity]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
   :dedent:

To exit the visualization, you can use ``ESC``.
