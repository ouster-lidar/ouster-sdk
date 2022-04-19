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

Notice that instead of taking a ``sample`` as we did in previous example, we used
:py:meth:`.Scans.stream`, which allows for a continuous live data stream.  We close the ``stream``
when we are finished, hence the use of :py:func:`.closing` in the highlighted line.

To exit the visualization, you can use ``ESC``.


.. _ex-pcap-to-csv:


Converting PCAPs to Other Formats
===================================

Sometimes we want to get a point cloud (``XYZ`` + other fields) as a ``CSV`` file for further
analysis with other tools.

To convert the first ``5`` scans of our sample data from a pcap file, you can try:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH pcap-to-csv --scan-num 5

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH pcap-to-csv --scan-num 5


The source code of an example below:

.. literalinclude:: /../python/src/ouster/sdk/examples/pcap.py
    :start-after: [doc-stag-pcap-to-csv]
    :end-before: [doc-etag-pcap-to-csv]
    :emphasize-lines: 33-37
    :linenos:
    :dedent:

Because we stored the scan as structured 2D images, we can easily recover it by loading it back into
a ``numpy.ndarray`` and continuing to use it as a 2D image.

.. code:: python

    import numpy as np

    # read array from CSV
    frame = np.loadtxt('my_frame_00000.csv', delimiter=',')

    # convert back to "fat" 2D image [H x W x 7] shape
    frame = frame.reshape((128, -1, frame.shape[1]))

We used ``128`` while restoring 2D image from a CSV file because it's the number of channels of our
``OS-1-128.pcap`` sample data recording.

Check :func:`.examples.pcap.pcap_to_csv` documentation for further details. For your convenience, we
have also provided :func:`examples.pcap.pcap_to_las` and :func:`examples.pcap.pcap_to_pcd`.
