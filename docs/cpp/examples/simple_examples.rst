============
C++ Examples
============

To facilitate working with the Ouster C++ SDK, we provide these examples of common operations. The
examples explained below are compiled into executables which print to screen to demonstratre
behavior. Build with ``BUILD_EXAMPLES`` and print to screen to demonstrate behavior.

Sensor Configuration
--------------------

In this example we show various ways to work with the sensor configuration
interface. The :cpp:type:`ouster::sensor::sensor_config` struct can be found
defined in ``types.h``. ``get_config`` and ``set_config`` are in ``client.h``.

To run this example:

.. code::

   config_example $SENSOR_HOSTNAME

If there are no errors, you should see a printout in five parts. 

Let's look at the first step, where we get the configuration of the sensor as it starts:

.. literalinclude:: /../examples/config_example.cpp
    :language: cpp
    :start-after: [doc-stag-cpp-get-config]
    :end-before: [doc-etag-cpp-get-config]
    :dedent:


The function ``get_config`` takes a ``sensor_config`` so we must declare it first. It returns true
if it succesfully retrieves the config, and false if an error occurs in connecting to the sensor and
setting the config. The function ``set_config`` works similarly, returning true if it successfully
sets the config, and false otherwise.

In the second step, we create a new config:

.. literalinclude:: /../examples/config_example.cpp
    :language: cpp
    :start-after: [doc-stag-cpp-make-config]
    :end-before: [doc-etag-cpp-make-config]
    :dedent:

The ``sensor_config`` struct consists of several ``std::optional`` members, which can be set
directly. Members which are not set will not set sensor configuration parameters when sent to the
sensor. In addition, there are two config flags, for automatic udp destination setting and config
persistence which can be set as above. The automatic udp destination flag cannot be set when
``config.udp_dest`` is set, as those conflict.


.. _ex-cpp-lidarscan:


Working with LidarScans
-----------------------

The :cpp:class:`ouster::LidarScan` is explained in depth conceptually in the `LidarScan reference
<reference/lidar-scan>`_. Here we cover some specifics that will be useful for C++ developers.


LidarScan constructors
++++++++++++++++++++++

Of foremost interest when using a :cpp:class:`LidarScan` is constructing a ``LidarScan`` which suits
your needs.

The simplest (and most common) method is to construct one to contain all data coming from your
sensor. You can do this by specifying the ``udp_proflie_lidar`` in your ``sensor_info``, or by
directly specifying the profile you care about.

For example, see the two snippets below:

.. literalinclude:: /../examples/lidar_scan_example.cpp
    :language: cpp
    :start-after: [doc-stag-lidarscan-profile-constructor]
    :end-before: [doc-etag-lidarscan-profile-constructor]
    :dedent:
 
.. literalinclude:: /../examples/lidar_scan_example.cpp
    :language: cpp
    :start-after: [doc-stag-lidarscan-dual-profile-constructor]
    :end-before: [doc-etag-lidarscan-dual-profile-constructor]
    :dedent:

But suppose you don't care about some of the data, such as the ambient and signal fields. You can
also specify to your LidarScan to only batch the relevant fields like so:

.. literalinclude:: /../examples/lidar_scan_example.cpp
    :language: cpp
    :start-after: [doc-stag-lidarscan-reduced-slots]
    :end-before: [doc-etag-lidarscan-reduced-slots]
    :dedent:

To see this in action, you can run the example executable ``lidar_scan_example``:

.. code::
    
    $ lidar_scan_example $SAMPLE_DUAL_RETURNS_PCAP $SAMPLE_DUAL_RETURNS_JSON

The source code of ``lidar_scan_example`` is available `here <https://github.com/ouster-lidar/ouster_example/blob/master/examples/lidar_scan_example.cpp>`_.


2D Representations and 3D representations
-----------------------------------------

.. todo:: 
    better to store the 2D and 3D code here and reference it from the LidarScan reference and just
    cover it twice

The core destaggering and projection to 3D capabilities are demonstrated in the
``representations_example`` executable.

Here we will cover slightly more sophisticated ways of working with the data also demonstrated in
that example.

To run this example:

.. code::
    
    representations_example $SAMPLE_DUAL_RETURNS_PCAP $SAMPLE_DUAL_RETURNS_JSON

The source code of ``representations_example`` is available `on the github <https://github.com/ouster-lidar/ouster_example/blob/master/examples/representations_example.cpp>`_.


Reshaping XYZ to 2D
+++++++++++++++++++

Users may find that they wish to access the ``x``, ``y``, and ``z`` coordinates of a single return
in a similar way. As the conversiton to cartesian coordinates returns an ``Eigen::Array`` ``n x 3``,
with ``n = w * h``, reshaping the resulting array is necessary. 

We can combine our knowledge in projecting into cartesian coordinates and destaggering using the
following function:

.. literalinclude:: /../examples/representations_example.cpp
    :language: cpp
    :start-after: [docs-stag-x-image-form]
    :end-before: [docs-etag-x-image-form]
    :dedent:

This demonstrates the functionality with ``x``, but it can be easily expanded to cover ``y`` and
``z`` as well.


Adjusting XYZLut With External Matrix
+++++++++++++++++++++++++++++++++++++

Users may find that they wish to apply an extra transform while projecting to cartesian coordinates.
Such a transform, likely an extrinsics matrix of some sort, can be baked into the
:cpp:struct:`ouster::XYZLut` created with :cpp:func:`ouster::make_xyz_lut`. 

In the following code, ``transformation`` represents the extrinsincs transform:

.. literalinclude:: /../examples/representations_example.cpp
    :language: cpp
    :start-after: [doc-stag-extrinsics-to-xyzlut]
    :end-before: [doc-etag-extrinsics-to-xyzlut]
    :dedent:


