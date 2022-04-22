.. _ex-lidar-scans:

=============================
The LidarScan Representation
=============================

.. contents::
   :local:
   :depth: 3

We provide the :py:class:`.LidarScan` class, which batches lidar packets by full rotations into
easily accessible fields of the appropriate type. The :py:class:`.LidarScan` also allows for easy
projection of the batched data into Cartesian coordinates, producing point clouds.

A :py:class:`.LidarScan` contains the fields specified at its initialization, queryable by accessing
``fields`` of the scan:

.. literalinclude:: /../python/src/ouster/sdk/examples/pcap.py
    :start-after: [doc-stag-pcap-query-scan]
    :end-before: [doc-etag-pcap-query-scan]
    :dedent:

You can run the above code on pcap containing packets of any type of :py:class:`.UDPProfileLidar`
with the :py:func:`.pcap.pcap_query_scan` example:


.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH query-scan

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH query-scan

On a packet containing dual returns data, you should note that your ``dtypes`` will not be
consistently ``uint32_t``, as you can tell from the result of running ``query-scan`` on dual returns
data:

.. code:: none

    Available fields and corresponding dtype in LidarScan
    RANGE           uint32
    RANGE2          uint32
    SIGNAL          uint16
    SIGNAL2         uint16
    REFLECTIVITY    uint8
    REFLECTIVITY2   uint8
    NEAR_IR         uint16


To change the available fields in a :py:class:`.LidarScan`, initialize the :py:class:`.LidarScan`
using :py:class:`.UDPProfileLidar` as follows:

.. code:: python

    LidarScan(h, w, metadata.format.udp_profile_lidar)


You can try it yourself with one of our :ref:`recorded dual returns data snippets<dual-returns-snippets>`!


.. _ex-staggered-and-destaggered:

Staggered vs Destaggered 2D Representations
=============================================

The default representation of a :py:class:`.LidarScan` stores data in **staggered** columns, meaning
that each column contains measurements taken at a single timestamp. As the lasers flashing at each
timestamp are arranged over several different azimuths, the resulting 2D image if directly
visualized is not a natural image. 

Let's take a look at a typical **staggered** representation:

.. figure:: /images/lidar_scan_staggered.png
   :align: center

   LidarScan ``RANGE`` field visualized with :py:func:`matplotlib.pyplot.imshow()` and simple gray
   color mapping for better look.

For the natural 2D image, we **destagger** the relevant field of the :py:class:`.LidarScan` with the
:py:func:`.client.destagger` function, changing the columns to represent azimuth intsead of
timestamp:

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


The above code gives the scene below, which we have magnified two patches for better visiblity.

.. figure:: /images/lidar_scan_destaggered.png
    :align: center

    **destaggered** LidarScan ``RANGE`` field

After destaggering, we can see the scene contains a man on a bicycle, a few cars, and many trees.
This image now makes visual sense, and we can easily use this data in common visual task pipelines.

.. todo:: 
    (Kai) Might be nice here or somewhere else to cover how to duplicate
    timestamps into an 'img', "destagger" it, and then use for for association
    of XYZ points with their timestamps 

.. note::

    UPDATE LINK!!! By the way, you can view this particular scene in both 2D and 3D at Ouster's
    `Data App`_! Use your mouse to click and move the 3D scene, and the listed controls to rotate
    between different destaggered image views.

.. _Data App: https://webslam.ouster.dev/slam/1610482355.9361048.rVdW_dgws/

.. _ex-xyzlut:


Projecting into Cartesian Coordinates
======================================

To facilitate working with 3D points, you can call :py:func:`.client.XYZLut` on the appropriate
metedata, creating a function which projects the ``RANGE`` and ``RANGE2`` fields into Cartesian
coordinates using a precomputed lookup table. The result of calling this function will be a point
cloud represented as a numpy array. See the API documentation for :py:func:`.client.XYZLut` for more
details.

.. literalinclude:: /../python/src/ouster/sdk/examples/client.py
    :start-after: [doc-stag-plot-xyz-points]
    :end-before: [doc-etag-plot-xyz-points]
    :emphasize-lines: 2-3
    :linenos:
    :dedent:

If you have a :ref:`configured<ex-configure-sensor>` sensor, you can run the above code excerpted
from the :py:func:`.client.plot_xyz_points` example with:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME plot-xyz-points

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME plot-xyz-points

That should open a 3D plot of a single scan of your location taken just now by your sensor. You
should be able to recognize the contours of the scene around you.

If you don’t have a sensor, you can run the same code with our
:py:func:`.pcap.pcap_display_xyz_points` pcap example:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH plot-xyz-points --scan-num 84

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH plot-xyz-points --scan-num 84


.. figure:: /images/lidar_scan_xyz_84.png
   :align: center

   Point cloud from OS1 sample data (scan 84). Points colored by ``SIGNAL`` value.


For visualizers which will stream consecutive frames from sensors or pcaps, check out our utilities
in :doc:`visualizations`.

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
