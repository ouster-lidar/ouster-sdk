====================================
ouster_client/include/ouster/types.h
====================================

.. contents::
   
Functions
=========

.. doxygenfunction:: ouster::sensor::default_sensor_info

.. doxygenfunction:: ouster::sensor::get_format

.. doxygenfunction:: ouster::sensor::n_cols_of_lidar_mode

.. doxygenfunction:: ouster::sensor::frequency_of_lidar_mode

Enums
=====

.. doxygenenum:: ouster::sensor::lidar_mode

.. doxygenenum:: ouster::sensor::timestamp_mode

.. doxygenenum:: ouster::sensor::OperatingMode

.. doxygenenum:: ouster::sensor::MultipurposeIOMode

.. doxygenenum:: ouster::sensor::Polarity


.. doxygenenum:: ouster::sensor::NMEABaudRate

.. doxygenenum:: ouster::sensor::UDPProfileLidar

.. doxygenenum:: ouster::sensor::UDPProfileIMU

.. doxygenenum:: ouster::sensor::client_state

.. doxygenenum:: ouster::sensor::ChanFieldType

.. doxygenenum:: ouster::sensor::ChanField

Structs
=======

.. doxygenstruct:: ouster::sensor::sensor_config
    :members:

.. doxygenstruct:: ouster::sensor::data_format
    :members:

.. doxygenstruct:: ouster::sensor::sensor_info
    :members:

Classes
=======

.. doxygenclass:: ouster::sensor::packet_format
    :members:

Types
=====

.. doxygentypedef:: ouster::img_t

.. doxygentypedef:: ouster::mat4d

.. doxygentypedef:: ouster::sensor::AzimuthWindow

.. doxygentypedef:: ouster::sensor::ColumnWindow

To String Functions
===================

.. doxygengroup:: ouster_sensor_types_to_string
    :content-only:

Of String Functions
===================

.. doxygengroup:: ouster_sensor_types_of_string
    :content-only:

Operators
=========

.. doxygengroup:: ouster_client_types_operators
    :content-only:
      
.. toctree::
   :caption: Ouster Client types.h
