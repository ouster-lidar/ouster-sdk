Module :mod:`ouster.sdk.sensor`
===============================

.. rubric:: ouster.sdk.sensor

.. autoexception:: ouster.sdk.sensor.ClientError
.. autoexception:: ouster.sdk.sensor.ClientTimeout
.. autoexception:: ouster.sdk.sensor.ClientOverflow

.. autofunction:: ouster.sdk.sensor.get_config

.. autofunction:: ouster.sdk.sensor.set_config

.. autoclass:: ouster.sdk.sensor.Sensor
   :members:
   :special-members: __iter__

.. autoclass:: ouster.sdk.sensor.SensorHttp
   :members:
   :special-members: __iter__

.. autoclass:: ouster.sdk.sensor.SensorPacketSource
   :members:
   :special-members: __iter__

.. autoclass:: ouster.sdk.sensor.SensorClient
   :members:
   :special-members: __iter__

.. autoclass:: ouster.sdk.sensor.SensorScanSource
   :members:
   :special-members: __iter__

.. rubric:: ouster.sdk.sensor.util

.. warning::

   **Deprecated since version 0.15.0**
   ``build_sensor_config`` is deprecated: manually build the configuration or use the SensorScanSource instead. 
   ``build_sensor_config`` will be removed in the upcoming release.

.. automodule:: ouster.sdk.sensor.util
   :members:
   :undoc-members:
   :show-inheritance:
