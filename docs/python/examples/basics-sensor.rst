.. _ex-basic-sensor:

=============================
Working with an Ouster sensor
=============================

.. contents::
   :local:
   :depth: 3


.. _ex-configure-sensor:


Configuring Your Sensor
========================
To work with your sensor, you should configure the ports, the :py:class:`.OperatingMode`, and the
:py:class:`.LidarMode`:

.. literalinclude:: /../python/src/ouster/sdk/examples/client.py
    :start-after: [doc-stag-configure]
    :end-before: [doc-etag-configure]
    :dedent:

Each config parameter corresponds directly to the sensor configuration parameters available on the
sensor.

You can run the above code, captured in the :py:func:`~.client.configure_sensor_params` example, as
follows:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME configure-sensor

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME configure-sensor

Once you've configured your sensor, you shouldn't have to configure it again until it shuts down or
restarts.  You can explore the ``persist`` flag to persist ``port`` and ``udp_dest`` settings over
sensor restarts.

If you have a Rev6 or later sensor and are running FW 2.2+, you should be able to configure your
sensor to use dual returns by setting the config parameter :py:class:`.UDPProfileLidar`:

.. literalinclude:: /../python/src/ouster/sdk/examples/client.py
    :start-after: [doc-stag-config-udp-profile]
    :end-before: [doc-etag-config-udp-profile]
    :dedent:


Try the :py:func:`~.client.configure_dual_returns` configuration example on your Rev6 or later
sensor:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME configure-dual-returns

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME configure-dual-returns


Obtaining Sensor Metadata
==========================

Ouster sensors require metadata to interpret the readings of the sensor. Represented by the object
:py:class:`.SensorInfo`, metadata fields include configuration parameters such as ``lidar_mode`` and
sensor intrinsics like ``beam_azimuth_angles``.

When you work with a sensor, the client will automatically fetch the metadata. Recorded
``pcaps``, however, must always be accompanied by a ``json`` file containing the metadata of the
sensor as it was when the data was recorded.

Since it's crucial to save the correct metadata file, let's see how we can get that from a sensor.
Try running the following example:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME fetch-metadata

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.client $SENSOR_HOSTNAME fetch-metadata


And now let's look inside the :py:func:`~.client.fetch_metadata` we just ran:

.. literalinclude:: /../python/src/ouster/sdk/examples/client.py
    :start-after: [doc-stag-fetch-metadata]
    :end-before: [doc-etag-fetch-metadata]
    :dedent:

Seems simple enough!
