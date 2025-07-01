types.h
=======

.. contents::
   :local:


Enums
-----

.. doxygenenum:: lidar_mode
   :project: cpp_api

.. doxygenenum:: timestamp_mode
   :project: cpp_api

.. doxygenenum:: timestamp_mode
   :project: cpp_api

.. doxygenenum:: OperatingMode
   :project: cpp_api

.. doxygenenum:: MultipurposeIOMode
   :project: cpp_api

.. doxygenenum:: Polarity
   :project: cpp_api

.. doxygenenum:: NMEABaudRate
   :project: cpp_api

.. doxygenenum:: ouster::sensor::UDPProfileLidar
   :project: cpp_api

.. doxygenenum:: ouster::sensor::UDPProfileIMU
   :project: cpp_api

.. doxygenenum:: ouster::sensor::FullScaleRange
   :project: cpp_api

.. doxygenenum:: ouster::sensor::ReturnOrder
   :project: cpp_api

.. doxygenenum:: ouster::sensor::ThermalShutdownStatus
   :project: cpp_api

.. doxygenenum:: ouster::sensor::ShotLimitingStatus
   :project: cpp_api

.. doxygenenum:: ouster::sensor::ChanFieldType
   :project: cpp_api

Variables
---------

.. doxygenvariable:: range_unit
   :project: cpp_api

.. doxygenvariable:: gen1_altitude_angles
   :project: cpp_api

.. doxygenvariable:: gen1_azimuth_angles
   :project: cpp_api

.. doxygenvariable:: default_imu_to_sensor_transform
   :project: cpp_api

.. doxygenvariable:: default_lidar_to_sensor_transform
   :project: cpp_api

Class
-----

.. doxygenclass:: ouster::sensor::packet_format
   :project: cpp_api
   :members:

.. doxygenclass:: ouster::sensor::product_info
   :project: cpp_api
   :members:

.. doxygenclass:: ouster::sensor::sensor_info
   :project: cpp_api
   :members:

Structs
-------

.. doxygenstruct:: ouster::sensor::calibration_status
   :project: cpp_api
   :members:

.. doxygenstruct:: ouster::sensor::data_format
   :project: cpp_api
   :members:

.. doxygenstruct:: ouster::sensor::sensor_config
   :project: cpp_api
   :members:

Namespaces
----------

.. doxygennamespace:: ChanField
   :project: cpp_api
   :members:

.. doxygennamespace:: ouster::sensor::ChanField
   :project: cpp_api
   :members:

Typedefs
--------

.. doxygentypedef:: ouster::img_t
   :project: cpp_api

.. doxygentypedef:: ouster::mat4d
   :project: cpp_api

.. doxygentypedef:: ouster::PointsD
   :project: cpp_api

.. doxygentypedef:: ouster::PointsF
   :project: cpp_api


Functions
---------

.. doxygenfunction:: ouster::sensor::to_string(const ouster::sensor::product_info&)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(lidar_mode)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::lidar_mode_of_string
   :project: cpp_api
   
.. doxygenfunction:: ouster::sensor::n_cols_of_lidar_mode
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::frequency_of_lidar_mode
   :project: cpp_api
   
.. doxygenfunction:: ouster::sensor::to_string(timestamp_mode)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::timestamp_mode_of_string
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(OperatingMode)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::operating_mode_of_string
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(MultipurposeIOMode)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::multipurpose_io_mode_of_string
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(Polarity)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::polarity_of_string
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(NMEABaudRate)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::nmea_baud_rate_of_string
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(AzimuthWindow)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(UDPProfileLidar)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::udp_profile_lidar_of_string
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(UDPProfileIMU)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::udp_profile_imu_of_string
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(FullScaleRange)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::full_scale_range_of_string
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::return_order_of_string
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(ReturnOrder)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(ShotLimitingStatus)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(ThermalShutdownStatus)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::check_signal_multiplier
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::metadata_from_json
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::parse_config
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(const ouster::sensor::sensor_config&)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(const ouster::sensor::calibration_status&)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::client_version
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::firmware_version_from_metadata
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::field_type_size
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::field_type_mask
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::to_string(ChanFieldType)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::get_format(const ouster::sensor::sensor_info&)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::get_format(UDPProfileLidar, size_t, size_t)
   :project: cpp_api