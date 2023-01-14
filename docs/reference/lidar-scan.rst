==============
Lidar Scan API
==============

In this reference document, we explain a core concept in both C++ and Python, and will often link
function names in the order (``C++ class/function``, ``Python class/function``). For
langauge-specific usage and running example code, please see the :ref:`Python LidarScan examples<ex-python-lidarscan>`
or the :ref:`C++ LidarScan examples<ex-cpp-lidarscan>`.

The ``LidarScan`` class (:cpp:class:`ouster::LidarScan`, :py:class:`.LidarScan`) batches lidar
packets by full rotations into accessible fields of the appropriate type. The ``LidarScan`` also
allows for easy projection of the batched data into Cartesian coordinates, producing point clouds.

.. todo:: 
    Update so that all code-tabs are actually literalincludes from examples somewhere
    Currently group tabs are broken since they do not match between tabs and code-tabs


Creating the LidarScan
======================

A ``LidarScan`` (:cpp:class:`ouster::LidarScan`, :py:class:`.LidarScan`) contains fields of data
specified at its initialization either through a lidar profile or a specific list of fields:

.. tabs::

    .. code-tab:: python

        scan = LidarScan(h, w, info.format.udp_profile_lidar)

    .. tab:: C++

        .. literalinclude:: /../examples/lidar_scan_example.cpp
            :language: cpp
            :start-after: [doc-stag-lidarscan-profile-constructor]
            :end-before: [doc-etag-lidarscan-profile-constructor]
            :dedent:

Since each ``LidarScan`` corresponds to a single frame (and is batched accordingly), you can access
the ``frame_id`` with simply:

.. tabs::

    .. code-tab:: python
        
        frame_id = scan.frame_id # frame_id is an int

    .. code-tab:: c++

        int32_t frame_id = scan.frame_id

In addition to ``frame_id`` and the fields specified at initialization, a ``LidarScan`` also
contains the column header information: ``timestamp``, ``status``, ``measurement_id``. These are
aggregated from each measurement block into W-element arrays, which are represented as an
``Eigen::Array`` and a ``numpy.ndarray`` in C++ and Python respectively. Note that if you use set
the sensor configuration parameter ``azimuth_window`` to something less than the full width, the
values in the header outside the azimuth window will be 0'd out accordingly.

.. tabs::

    .. code-tab:: python

        # each of these has as many entries as there are columns in the scan
        ts_0 = scan.timestamp[0]
        measurement_id_0 = scan.measurement_id[0]
        status_0 = status[0]
        
        
    .. tab:: C++

        .. literalinclude:: /../examples/lidar_scan_example.cpp
            :language: cpp
            :start-after: [doc-stag-lidarscan-cpp-headers]
            :end-before: [doc-etag-lidarscan-cpp-headers]
            :dedent:

For any field contained by a ``LidarScan`` scan, you can access that field in the following way:

.. tabs::

    .. code-tab:: python

        ranges = scan.field(client.ChanField.RANGE)
        ranges2 = scan.field(client.ChanField.RANGE2)
        reflectivity = scan.field(client.ChanField.REFLECTIVITY)
        reflectivity2 = scan.field(client.ChanField.REFLECTIVITY2)
    
    .. tab:: C++

        .. literalinclude:: /../examples/lidar_scan_example.cpp
            :language: cpp
            :start-after: [doc-stag-lidarscan-cpp-fields]
            :end-before: [doc-etag-lidarscan-cpp-fields]
            :dedent:

Finally, the fields of an existing ``LidarScan`` can be found by accessing the ``fields`` of the
scan through an iterator:

.. tabs::

    .. tab:: Python

        .. literalinclude:: /../python/src/ouster/sdk/examples/pcap.py
            :language: python
            :start-after: [doc-stag-pcap-query-scan]
            :end-before: [doc-etag-pcap-query-scan]
            :dedent:

    .. tab:: C++

        .. literalinclude:: /../examples/lidar_scan_example.cpp
            :language: cpp
            :start-after: [doc-stag-cpp-scan-iter]
            :end-before: [doc-etag-cpp-scan-iter]
            :dedent:

.. note::
    
    The units of a particular field from a ``LidarScan`` are consistent even
    when you use lidar profiles which scale the returned data from the sensor.
    This is because the LidarScan will reverse the scaling for you when
    parsing. For example, the RANGE field on a LidarScan constructed with the
    low data rate profile will be in millimeters even though the return from
    the sensor is given in 8mm increments.

Running the above code on a sample ``LidarScan`` will give you output that looks like:

.. include:: /python/examples/lidar-scan.rst
    :start-after: [start-query-scan-result]
    :end-before: [end-query-scan-result]

Now that we know how to create the ``LidarScan`` and access its contents, let's see what
we can do with it!


Projecting into Cartesian Coordinates
=====================================

To facilitate working with 3D points, you can create a pre-computed XYZ look-up table ``XYZLut``
(:cpp:struct:`ouster::XYZLut`, :py:func:`.client.XYZLut`) on the appropriate metadata, which you can
then use to project the ``RANGE`` and ``RANGE2`` fields into Cartesian coordinates efficiently. The
result of calling this function will be a point cloud represented as an array (either an
``Eigen::Array`` or a numpy array). See the respective API documentations for
:cpp:func:`ouster::cartesian` and :py:func:`.client.XYZLut` for more details.

.. tabs::

    .. tab:: Python

        .. literalinclude:: /../python/src/ouster/sdk/examples/client.py
            :language: python
            :start-after: [doc-stag-plot-xyz-points]
            :end-before: [doc-etag-plot-xyz-points]
            :dedent:

    .. tab:: C++

        .. literalinclude:: /../examples/representations_example.cpp
            :language: cpp
            :start-after: [doc-stag-cpp-xyz]
            :end-before: [doc-etag-cpp-xyz]
            :dedent:

Once you have your x, y, and z values, you can plot them easily to get something like:

.. figure:: /images/lidar_scan_xyz_84.png
   :align: center

   Point cloud from OS1 sample data (scan 84). Points colored by ``SIGNAL`` value.

To see how to generate this image, see the :ref:`Python LidarScan examples<ex-python-lidarscan>`.

Staggering and Destaggering
===========================

The default representation of ``LidarScan`` stores data in **staggered** columns, meaning
that each column contains measurements taken at a single timestamp. As the lasers flashing at each
timestamp are arranged over several different azimuths, the resulting 2D image if directly
visualized is not a natural image. 

Let's take a look at a typical **staggered** representation:

.. figure:: /images/lidar_scan_staggered.png
   :align: center

   LidarScan ``RANGE`` field visualized with :py:func:`matplotlib.pyplot.imshow()` and simple gray
   color mapping for better look.

It would be convenient to obtain a representation where the columns represent azimuth angle instead
of timestamp. For this natural 2D image, we *destagger* the relevant field of the ``LidarScan`` with
``destagger`` (:cpp:func:`ouster::destagger`, :py:func:`.client.destagger` function):

.. todo:: fix destagger for cpp link

.. tabs::

    .. code-tab:: python
        
        ranges = scan.field(client.ChanField.REFLECTIVITY)
        ranges_destaggered = client.destagger(source.metadata, range)

    .. tab:: C++

        .. literalinclude:: /../examples/representations_example.cpp
            :language: cpp
            :start-after: [doc-stag-cpp-destagger]
            :end-before: [doc-etag-cpp-destagger]
            :dedent:

The above code gives the scene below (see the long strip at the bottom). We've magnified two patches
for better visiblity atop.

.. figure:: /images/lidar_scan_destaggered.png
    :align: center

    **destaggered** LidarScan ``RANGE`` field

After *destaggering*, we can see the scene contains a man on a bicycle, a few cars, and many trees.
This image now makes visual sense, and we can easily use this data in common visual task pipelines.


Populating LidarScans
=====================

This reference has covered how to a ``LidarScan``, and how to access, project, and destagger its
contents. But in order for ``LidarScan`` s to be useful, we need a way to populate them with packet
data! For convenience, the Ouster Python SDK provides the :py:class:`.Scans` interface, which allows
both sampling, used in :ref:`ex-visualization-with-matplotlib`, and streaming, used in
:ref:`ex-stream`.

Under the hood, this class batches packets into ``LidarScans``. C++ users must batch packets
themselves using the :cpp:class:`ouster::ScanBatcher` class. To get a feel for how to use it, we recommend
reading `this example on Github
<https://github.com/ouster-lidar/ouster_example/blob/master/examples/client_example.cpp#L93>`_.
