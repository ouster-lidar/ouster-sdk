Using the API
=============

In this example we show various ways to work with the sensor configuration interface. 

.. rubric:: Imports

First let's add the necessary imports to work with the sensor configuration API.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :language: python
         :start-after: [doc-stag-config-imports]
         :end-before: [doc-etag-config-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :dedent: 0
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/config_example.cpp
         :language: cpp
         :start-after: [doc-stag-cpp-config-imports]
         :end-before: [doc-etag-cpp-config-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/config_example.cpp>`__
         :dedent: 0


Get Configuration
-----------------

Let’s look at the first step, where we get the configuration of the sensor as it starts:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :language: python
         :start-after: [doc-stag-get-config]
         :end-before: [doc-etag-get-config]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :dedent: 4
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/config_example.cpp
         :language: cpp
         :start-after: [doc-stag-cpp-get-config]
         :end-before: [doc-etag-cpp-get-config]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/config_example.cpp>`__
         :dedent: 4


:ouster:func:`get_config <py=ouster.sdk.sensor.get_config|cpp=ouster::sdk::sensor::get_config>`
takes the sensor hostname and returns a
:ouster:class:`SensorConfig <py=ouster.sdk.core.SensorConfig|cpp=ouster::sdk::core::SensorConfig>` directly;
it throws if it can't connect to the sensor or retrieve the config.

In the full C++ example (click the "View on GitHub" on the top-right corner of the code snippet above), this is the first of five numbered
steps; if there are no errors, running it end-to-end prints a status line for each step. The full
Python example follows the same get-config, modify, set-config, re-fetch flow without the numbered
output.

The sensor configuration can be updated using the API which will be covered in the next sections.

The configuration parameters provided are in direct correspondence with the settings available for the sensor. 
A complete list of the sensor settings can be found in the `sensor configuration overview <https://static.ouster.dev/sensor-docs/image_route1/image_route2/common_sections/API/sensor_configuration_description.html>`__ .

Some of the most commonly modified parameters are described below.

Configure Dual Returns
^^^^^^^^^^^^^^^^^^^^^^

Once you've configured your sensor, you shouldn't have to configure it again until it shuts down or
restarts.  You can explore the ``persist`` flag to persist ``port`` and ``udp_dest`` settings over
sensor restarts.

If you have a Rev6 or later sensor and are running FW 2.2+, you should be able to configure your
sensor to use dual returns by setting the config parameter
:ouster:enum:`UDPProfileLidar <py=ouster.sdk.core.UDPProfileLidar|cpp=ouster::sdk::core::UDPProfileLidar>`.


.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :language: python
         :start-after: [doc-stag-config-udp-profile]
         :end-before: [doc-etag-config-udp-profile]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :dedent: 4
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/config_example.cpp
         :language: cpp
         :start-after: [doc-stag-config-udp-profile]
         :end-before: [doc-etag-config-udp-profile]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/config_example.cpp>`__
         :dedent: 4

Timing Configuration
---------------------

The sensor's timing and synchronization can be configured using the following options:

Timing Source: The ``timestamp_mode`` field allows you to switch between internal timing, PTP 1588, or an external sync-pulse (PPS) input.

GNSS Input: The ``multipurpose_io_mode`` can be set to accept NMEA sentences from a GNSS receiver via the multi-purpose IO port. Other modes are available for driving sync or PPS outputs.

NMEA Baud Rate: The ``nmea_baud_rate`` parameter sets the serial speed for the NMEA UART, which must correspond to the baud rate of your GNSS receiver (e.g., 9600, 115200).


.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :language: python
         :start-after: [doc-stag-config-timing]
         :end-before: [doc-etag-config-timing]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :dedent: 4
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/config_example.cpp
         :language: cpp
         :start-after: [doc-stag-config-timing]
         :end-before: [doc-etag-config-timing]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/config_example.cpp>`__
         :dedent: 4


For a complete list of configurable options view API reference :ouster:class:`SensorConfig <py=ouster.sdk.core.SensorConfig|cpp=ouster::sdk::core::SensorConfig>`.
To view the complete code for above example please refer to the github repository. The github icon on the top-right corner on each code snippet will redirect you to the actual code on the repository.

.. _set-configuration:

Set Configuration
-----------------

:ouster:func:`set_config <py=ouster.sdk.sensor.set_config|cpp=ouster::sdk::sensor::set_config>`  can be used to apply sensor configuration parameters.

The :ouster:class:`SensorConfig <py=ouster.sdk.core.SensorConfig|cpp=ouster::sdk::core::SensorConfig>` struct consists of several optional members, which can be set
directly. Members which are not set will not set sensor configuration parameters when sent to the
sensor.

To work with your sensor, you should configure the ports, the :ouster:enum:`OperatingMode <py=ouster.sdk.core.OperatingMode|cpp=ouster::sdk::core::OperatingMode>`, and the
:ouster:enum:`LidarMode <py=ouster.sdk.core.LidarMode|cpp=ouster::sdk::core::LidarMode>`.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :language: python
         :start-after: [doc-stag-make-config]
         :end-before: [doc-etag-make-config]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :dedent: 4
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/config_example.cpp
         :language: cpp
         :start-after: [doc-stag-cpp-make-config]
         :end-before: [doc-etag-cpp-make-config]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/config_example.cpp>`__
         :dedent: 4


Once defined, one can now call :ouster:func:`set_config <py=ouster.sdk.sensor.set_config|cpp=ouster::sdk::sensor::set_config>` to apply the above configuration.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/core.py
         :language: python
         :start-after: [doc-stag-set-config]
         :end-before: [doc-etag-set-config]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/core.py>`__
         :dedent: 4
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/config_example.cpp
         :language: cpp
         :start-after: [doc-stag-cpp-set-config]
         :end-before: [doc-etag-cpp-set-config]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/config_example.cpp>`__
         :dedent: 4


.. note::
   
   The automatic udp destination flag cannot be set via CONFIG_UDP_DEST_AUTO when
   ``config.udp_dest`` is set, as those conflict.

After setting the configuration, retrieve the configuration from the sensor using :ouster:func:`get_config <py=ouster.sdk.sensor.get_config|cpp=ouster::sdk::sensor::get_config>`
to ensure that the settings were applied correctly.
