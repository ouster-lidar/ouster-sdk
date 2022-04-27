=======
types.h
=======

.. contents::
    :local:

Type-defs
=========

.. doxygentypedef:: ouster::img_t

.. doxygentypedef:: ouster::mat4d

.. doxygentypedef:: ouster::sensor::AzimuthWindow

.. doxygentypedef:: ouster::sensor::ColumnWindow

Helper Data
===========

Lidar Mode
----------
.. doxygenenum:: ouster::sensor::lidar_mode

.. doxygenfunction:: ouster::sensor::n_cols_of_lidar_mode

.. doxygenfunction:: ouster::sensor::frequency_of_lidar_mode

.. doxygenfunction:: ouster::sensor::to_string(lidar_mode mode)

.. doxygenfunction:: ouster::sensor::lidar_mode_of_string

Timestamp Mode
--------------

.. doxygenenum:: ouster::sensor::timestamp_mode

.. doxygenfunction:: ouster::sensor::to_string(timestamp_mode mode)

.. doxygenfunction:: ouster::sensor::timestamp_mode_of_string

Operating Mode
--------------

.. doxygenenum:: ouster::sensor::OperatingMode

.. doxygenfunction:: ouster::sensor::to_string(OperatingMode mode)

.. doxygenfunction:: ouster::sensor::operating_mode_of_string

Multipurpose IO Mode
--------------------

.. doxygenenum:: ouster::sensor::MultipurposeIOMode

.. doxygenfunction:: ouster::sensor::to_string(MultipurposeIOMode mode)

.. doxygenfunction:: ouster::sensor::multipurpose_io_mode_of_string


Polarity
--------

.. doxygenenum:: ouster::sensor::Polarity

.. doxygenfunction:: ouster::sensor::to_string(Polarity mode)

.. doxygenfunction:: ouster::sensor::polarity_of_string


NMEA Baud Rate
--------------

.. doxygenenum:: ouster::sensor::NMEABaudRate

.. doxygenfunction:: ouster::sensor::to_string(NMEABaudRate mode)

.. doxygenfunction:: ouster::sensor::nmea_baud_rate_of_string


UDP Profile Lidar
-----------------

.. doxygenenum:: ouster::sensor::UDPProfileLidar

.. doxygenfunction:: ouster::sensor::to_string(UDPProfileLidar mode)

.. doxygenfunction:: ouster::sensor::udp_profile_lidar_of_string


UDP Profile IMU
---------------

.. doxygenenum:: ouster::sensor::UDPProfileIMU

.. doxygenfunction:: ouster::sensor::to_string(UDPProfileIMU mode)

.. doxygenfunction:: ouster::sensor::udp_profile_imu_of_string

Chan Field
---------------

.. doxygenenum:: ouster::sensor::ChanFieldType

.. doxygenenum:: ouster::sensor::ChanField

.. doxygenfunction:: ouster::sensor::to_string(ChanField field)

Sensor Info
===========

.. doxygenstruct:: ouster::sensor::sensor_info
    :members:

.. doxygenfunction:: ouster::sensor::default_sensor_info

.. doxygenfunction:: ouster::sensor::parse_metadata

.. doxygenfunction:: ouster::sensor::metadata_from_json

.. doxygenfunction:: ouster::sensor::convert_to_legacy

.. doxygenfunction:: ouster::sensor::operator==(const sensor_info& lhs, const sensor_info& rhs)

.. doxygenfunction:: ouster::sensor::operator!=(const sensor_info& lhs, const sensor_info& rhs)

.. doxygenfunction:: ouster::sensor::to_string(const sensor_info& info)

Sensor Config
=============

.. doxygenstruct:: ouster::sensor::sensor_config
    :members:

.. doxygenfunction:: ouster::sensor::parse_config(const std::string& config)

.. doxygenfunction:: ouster::sensor::operator==(const sensor_config& lhs, const sensor_config& rhs)

.. doxygenfunction:: ouster::sensor::operator!=(const sensor_config& lhs, const sensor_config& rhs)

.. doxygenfunction:: ouster::sensor::to_string(const sensor_config& config)

Data Format
===========

.. doxygenstruct:: ouster::sensor::data_format
    :members:

.. doxygenfunction:: ouster::sensor::operator==(const data_format& lhs, const data_format& rhs)

.. doxygenfunction:: ouster::sensor::operator!=(const data_format& lhs, const data_format& rhs)

Packet Format
=============

.. doxygenclass:: ouster::sensor::packet_format
    :members:

.. doxygenfunction:: ouster::sensor::get_format

MISC
====

.. doxygenfunction:: ouster::sensor::client_version

.. doxygenvariable:: ouster::sensor::range_unit
