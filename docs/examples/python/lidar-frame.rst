.. _ex-python-lidarframe:

The LidarFrame Representation
============================


The :py:class:`.core.LidarFrame` class is explained in depth in the :doc:`/features/consumption/lidar_frame`, which we recommend reading.

We provide example code to aid in understanding.

Query frame for available fields
---------------------------------

To run the sample code which queries fields from a frame, use the :py:func:`.pcap.pcap_query_frame` example:

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. code::

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH query-frame

   .. tab-item:: Windows x64
      :sync: powershell

      .. code::

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH query-frame

You can run the above on pcaps containing packets of any type of :py:class:`.core.UDPProfileLidar`.

You might notice that on a pcap containing dual returns data, you should note
that your ``dtypes`` will not be consistently ``uint32_t``, as you can tell
from the result of running ``query-frame`` on dual returns data:

..
   [start-query-frame-result]

.. code:: none

    Available fields and corresponding dtype in LidarFrame
    RANGE           uint32
    RANGE2          uint32
    SIGNAL          uint16
    SIGNAL2         uint16
    REFLECTIVITY    uint8
    REFLECTIVITY2   uint8
    NEAR_IR         uint16

..
   [end-query-frame-result]


.. _ex-xyzlut:


Projecting into Cartesian Coordinates
-------------------------------------

Let's plot some points!

If you have a :ref:`configured<ex-configure-sensor>` sensor, you can plot some points in XYZ using the :py:func:`~.examples.core.plot-xyz-points` example:

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. code::

        $ python3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME plot-xyz-points

   .. tab-item:: Windows x64
      :sync: powershell

      .. code::

        PS > py -3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME plot-xyz-points

That should open a 3D plot of a single frame of your location taken just now by
your sensor. You should be able to recognize the contours of the scene around
you.

If you don’t have a sensor, you can run the same code with our
:py:func:`.pcap.pcap_display_xyz_points` pcap example:

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. code::

        $ python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH plot-xyz-points --frame-num 84

   .. tab-item:: Windows x64
      :sync: powershell

      .. code::

        PS > py -3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH plot-xyz-points --frame-num 84

For visualizers which will stream consecutive frames from sensors or pcaps,
check out our utilities in :doc:`visualizations`.

.. _ex-correlating-2d-and-3d:


Working with 2D and 3D Representations Simultaneously
------------------------------------------------------

The direct correlation between 2D and 3D representations in an Ouster sensor provides a powerful
framework for working with the data. As an easy example, you might decide you want to look at only
the 3D points within a certain range and from certain azimuth angles.

.. literalinclude:: /../python/src/ouster/sdk/examples/core.py
    :start-after: [doc-stag-filter-3d]
    :end-before: [doc-etag-filter-3d]
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

.. tab-set::
   :sync-group: os-commands

   .. tab-item:: Linux/macOS
      :sync: unix

      .. code::

        $ python3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME filter-3d-by-range-and-azimuth

   .. tab-item:: Windows x64
      :sync: powershell

      .. code::

        PS > py -3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME filter-3d-by-range-and-azimuth
