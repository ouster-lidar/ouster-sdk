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
    :start-after: [doc-stag-cpp-get-config]
    :end-before: [doc-etag-cpp-get-config]
    :dedent:


The function ``get_config`` takes a ``sensor_config`` so we must declare it first. It returns true
if it succesfully retrieves the config, and false if an error occurs in connecting to the sensor and
setting the config. The function ``set_config`` works similarly, returning true if it successfully
sets the config, and false otherwise.

In the second step, we create a new config:

.. literalinclude:: /../examples/config_example.cpp
    :start-after: [doc-stag-cpp-make-config]
    :end-before: [doc-etag-cpp-make-config]
    :dedent:

The ``sensor_config`` struct consists of several ``std::optional`` members, which can be set
directly. Members which are not set will not set sensor configuration parameters when sent to the
sensor. In addition, there are two config flags, for automatic udp destination setting and config
persistence which can be set as above. The automatic udp destination flag cannot be set when
``config.udp_dest`` is set, as those conflict.

Working with LidarScans
-----------------------

The :cpp:class:`ouster::LidarScan` class, implemented in ``ouster_client``, is a convenience class for
efficient operations on aggregated lidar data. In this example, we cover the
various constructors and batching data into LidarScans.

To run this example:

.. code::
    
    lidar_scan_example $SAMPLE_DUAL_RETURNS_PCAP $SAMPLE_DUAL_RETURNS_JSON


LidarScan constructors
++++++++++++++++++++++
The :cpp:class:`LidarScan` class has several constructors. Of foremost interest is constructing the
``LidarScan`` which contains all data coming from your sensor. You can do this by specifying the
``udp_proflie_lidar`` in your ``sensor_info``, or by directly specifying the profile you care about.

.. literalinclude:: /../examples/lidar_scan_example.cpp
    :start-after: [doc-stag-lidarscan-constructor]
    :end-before: [doc-etag-lidarscan-constructor]
    :dedent:

 
But suppose you don't care about some of the data, such as the ambient and signal fields. You can
also specify to your LidarScan to only batch the relevant fields like so:


.. literalinclude:: /../examples/lidar_scan_example.cpp
    :start-after: [doc-stag-lidarscan-reduced-slots]
    :end-before: [doc-etag-lidarscan-reduced-slots]
    :dedent:


LidarScan fields and headers
++++++++++++++++++++++++++++
Now let's look at how to access the relevant information from a ``LidarScan`` when it's relevant:

The column header information on each measurement block is aggregated in the scan and accessible as
a W-element vector. Note that if you use an azimuth_window, then those values in the header outside
the azimuth window will be 0'd out accordingly.

.. literalinclude:: /../examples/lidar_scan_example.cpp
    :start-after: [doc-stag-lidarscan-fields-headers]
    :end-before: [doc-etag-lidarscan-fields-headers]
    :dedent:

Finally, when you're working with a ``LidarScan``, you may wish to iterate through the fields. You can
do so with an iterator:

.. literalinclude:: /../examples/lidar_scan_example.cpp
    :start-after: [doc-stag-cpp-lidarscan-iter]
    :end-before: [doc-etag-cpp-lidarscan-iter]
    :dedent:


2D Representations and 3D representations
------------------

.. todo :: Finish section

To run this example:

.. code::
    
    representations_example $SAMPLE_DUAL_RETURNS_PCAP $SAMPLE_DUAL_RETURNS_JSON


Getting XYZ Coordinates
+++++++++++++++++++++++
Now we're reading to look more closely at getting XYZ coordinates

.. literalinclude:: /../examples/lidar_scan_example.cpp
    :start-after: [doc-stag-cpp-xyz]
    :end-before: [doc-etag-cpp-xyz]
    :dedent:

Adjusting XYZLut With External Matrix
+++++++++++++++++++++++++++++++++++++


Destaggering a field
++++++++++++++++++++



Reshaping XYZ to 2D
+++++++++++++++++++

Combining our knowledge in obtaining XYZ coordinates and destaggering example code:

ex: get_x_in_image_form

LidarScan fields are w x h Eigen arrays where returns are accessible by their row and column index.
For example, the reflectivity return from the 10th beam at the 500th column would be accessible at
reflectivity(10, 500).
 
Users may find that they wish to access the x, y, and z coordinates of a single return in a similar
way. As the conversiton to xyz returns an Eigen array of n x 3, with n = w * h, reshaping the
resulting values is necessary. This function demonstrates that capability given a LidarScan for the
x coordinate, with y and z easily derivable using their respective columns of the cloud.


