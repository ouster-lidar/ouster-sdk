.. _ex-python-lidarscan:

============================
The LidarScan Representation
============================

.. contents::
   :local:
   :depth: 3

The :py:class:`.LidarScan` class is explained in depth in the :doc:`/reference/lidar-scan`, which we recommend reading.

We provide example code to aid in understanding.

To run the sample code which queries fields from a scan, use the :py:func:`.pcap.pcap_query_scan` example:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH query-scan

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH query-scan

You can run the above on pcaps containing packets of any type of :py:class:`.UDPProfileLidar`.

You might notice that on a pcap containing dual returns data, you should note
that your ``dtypes`` will not be consistently ``uint32_t``, as you can tell
from the result of running ``query-scan`` on dual returns data:

..
   [start-query-scan-result]

.. code:: none

    Available fields and corresponding dtype in LidarScan
    RANGE           uint32
    RANGE2          uint32
    SIGNAL          uint16
    SIGNAL2         uint16
    REFLECTIVITY    uint8
    REFLECTIVITY2   uint8
    NEAR_IR         uint16

..
   [end-query-scan-result]

.. _ex-staggered-and-destaggered:

Staggered vs Destaggered 2D Representations
===========================================

To generate staggered and destaggered images yourself, you can try the following sample code:

.. code:: python

    import matplotlib.pyplot as plt
    from more_itertools import nth

    # ... `metadata` and `source` variables created as in the previous examples

    scans = client.Scans(source)

    # iterate `scans` and get the 84th LidarScan (it can be different with your data)
    scan = nth(scans, 84)
    ranges = scan.field(client.ChanField.RANGE)

    # destagger ranges, notice `metadata` use, that is needed to get
    # sensor intrinsics and correctly data transforms
    ranges_destaggered = client.destagger(source.metadata, ranges)

    plt.imshow(ranges_destaggered, cmap='gray', resample=False)

.. todo:: 
    (Kai) Might be nice to cover either here or in the reference how to
    duplicate timestamps into an 'img', "destagger" it, and then use for for
    association of XYZ points with their timestamps 


.. _ex-xyzlut:


Projecting into Cartesian Coordinates
======================================

Let's plot some points!

If you have a :ref:`configured<ex-configure-sensor>` sensor, you can plot some points in XYZ using the :py:func:`.client.plot_xyz_points` example:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME plot-xyz-points

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME plot-xyz-points

That should open a 3D plot of a single scan of your location taken just now by
your sensor. You should be able to recognize the contours of the scene around
you.

If you don’t have a sensor, you can run the same code with our
:py:func:`.pcap.pcap_display_xyz_points` pcap example:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH plot-xyz-points --scan-num 84

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH plot-xyz-points --scan-num 84

For visualizers which will stream consecutive frames from sensors or pcaps,
check out our utilities in :doc:`visualizations`.

.. _ex-correlating-2d-and-3d:


Working with 2D and 3D Representations Simultaneously
======================================================

The direct correlation between 2D and 3D representations in an Ouster sensor provides a powerful
framework for working with the data. As an easy example, you might decide you want to look at only
the 3D points within a certain range and from certain azimuth angles.

.. literalinclude:: /../python/src/ouster/sdk/examples/client.py
    :start-after: [doc-stag-filter-3d]
    :end-before: [doc-etag-filter-3d]
    :emphasize-lines:  10-11, 15
    :linenos:
    :dedent:

Since we'd like to filter on azimuth angles, first we first destagger both the 2D and 3D points, so
that our columns in the ``HxW`` representation correspond to azimuth angle, not timestamp. (See
:ref:`ex-staggered-and-destaggered` for an explanation on destaggering.)

Then we filter the 3D points ``xyz_destaggered`` by comparing the range measurement to
``range_min``, which we can do because there is a 1:1 correspondence between the columns and rows of
the destaggered representations of ``xyz_destaggered`` and ``range_destaggered``. (Similarly, there
would be a 1:1 correspondence between the staggered representations ``xyz`` and ``range``, where the
columns correspond with timestamp).

Finally, we select only the azimuth columns we're interested in. In this case, we've arbitrarily
chosen the first 270 degrees of rotation.

If you have a :ref:`configured<ex-configure-sensor>` sensor, you can run this code with an example:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME filter-3d-by-range-and-azimuth

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME filter-3d-by-range-and-azimuth
