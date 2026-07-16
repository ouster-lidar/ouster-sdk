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

We have covered the api to configure your sensor in detail at :doc:`sensor configuration page </features/sensor_config/using-the-api>`.

The process can be demonstrated directly via built examples in Ouster SDK, via :py:func:`~.examples.core.configure_sensor_params` example, as follows:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME configure-sensor

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME configure-sensor


Try the :py:func:`~.examples.configure_dual_returns` configuration example on your Rev6 or later
sensor:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME configure-dual-returns

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME configure-dual-returns


Obtaining Sensor Metadata
==========================

Ouster sensors require metadata to interpret the readings of the sensor. Represented by the object
:py:class:`.core.SensorInfo`, metadata fields include configuration parameters such as ``lidar_mode`` and
sensor intrinsics like ``beam_azimuth_angles``.

When you work with a sensor, the client will automatically fetch the metadata. Recorded
``pcaps``, however, must always be accompanied by a ``json`` file containing the metadata of the
sensor as it was when the data was recorded.

Since it's crucial to save the correct metadata file, let's see how we can get that from a sensor.
Try running the following example:

.. tabs::

    .. code-tab:: console Linux/macOS

        $ python3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME fetch-metadata

    .. code-tab:: powershell Windows x64

        PS > py -3 -m ouster.sdk.examples.core $SENSOR_HOSTNAME fetch-metadata

Seems simple enough!
