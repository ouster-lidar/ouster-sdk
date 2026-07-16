
.. _ex-cpp-simple-examples:

Basic Examples
==============

To facilitate working with the Ouster C++ SDK, we provide these examples of common operations. The
examples explained below are compiled into executables which print to screen to demonstrate
behavior. Build with ``BUILD_EXAMPLES`` and print to screen to demonstrate behavior.

.. _ex-cpp-sensor-config:

Sensor Configuration
--------------------

The :doc:`Sensor config page </features/sensor_config/using-the-api>` covers various ways to work with the sensor configuration interface.

The concepts explained there are demonstrated in a compiled example that you can run using the command below:

.. code::

   config_example $SENSOR_HOSTNAME


.. _ex-cpp-lidarframe:


LidarFrame constructors
----------------------

We learnt all about ``LidarFrames`` in :doc:`/features/consumption/lidar_frame`.

Here we will see this in action, you can do this by running the example executable ``lidar_frame_example``:

.. code::
    
    $ lidar_frame_example $SAMPLE_DUAL_RETURNS_PCAP $SAMPLE_DUAL_RETURNS_JSON

The source code of ``lidar_frame_example`` is available `here <https://github.com/ouster-lidar/ouster-sdk/blob/master/examples/lidar_frame_example.cpp>`_.


.. _ex-cpp-representations:

2D Representations and 3D representations
-----------------------------------------

The core destaggering and projection to 3D capabilities are demonstrated in the
``representations_example`` executable. The concepts demonstrated by this example are covered in detail in
:doc:`LidarFrame processing section </features/processing/using-the-api>`.


Here we will cover slightly more sophisticated ways of working with the data also demonstrated in
that example.

To run this example:

.. code::
    
    representations_example $SAMPLE_DUAL_RETURNS_PCAP $SAMPLE_DUAL_RETURNS_JSON

The source code of ``representations_example`` is available `on the GitHub <https://github.com/ouster-lidar/ouster-sdk/blob/master/examples/representations_example.cpp>`_.
