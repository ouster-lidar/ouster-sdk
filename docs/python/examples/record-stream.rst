.. _ex-record-stream-viz:

=====================================
Recording, Streaming, and Conversion
=====================================

.. contents::
   :local:
   :depth: 3


Recording Sensor Data
======================

It's easy to record data to a pcap file from a sensor programatically. Let's try it on a
:ref:`configured<ex-configure-sensor>` sensor:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME record-pcap

    .. code-tab:: powershell Windows x64

        PS >  py -3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME record-pcap


This will capture the :class:`.client.LidarPacket`'s and :class:`.client.ImuPacket`'s data for 10
seconds and store the pcap file along with the metadata json file into the current directory.

The source code of an example below:

.. literalinclude:: /../python/src/ouster/sdk/examples/client.py
   :start-after: [doc-stag-pcap-record]
   :end-before: [doc-etag-pcap-record]
   :emphasize-lines: 15
   :linenos:
   :dedent:

Good! The resulting pcap and json files can be used with any examples in the :mod:`.examples.pcap`
module.

.. _ex-stream:

Streaming Live Data
====================

Instead of working with a recorded dataset or a few captured frames of data, let's see if we can get
a live feed from your :ref:`configured<ex-configure-sensor>` sensor:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME live-plot-signal

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME live-plot-signal

This should give you a live feed from your sensor that looks like a black and white moving image.
Try waving your hand or moving around to find yourself within the image!

So how did we do that?

.. literalinclude:: /../python/src/ouster/sdk/examples/client.py
   :start-after: [doc-stag-live-plot-signal]
   :end-before: [doc-etag-live-plot-signal]
   :emphasize-lines: 2-3
   :linenos:
   :dedent:

Notice that instead of taking a ``sample``, we used :py:meth:`.Scans.stream`, which allows for a
continuous live data stream.  We close the ``stream`` when we are finished, hence the use of
:py:func:`.closing` in the highlighted line.

To exit the visualization, you can use ``ESC``.
