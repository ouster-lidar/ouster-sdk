=====================================
ouster_client/include/ouster/client.h
=====================================

.. contents::

Functions
=========

.. doxygengroup:: ouster_client_init
    :content-only:

.. doxygenfunction:: ouster::sensor::get_metadata

.. doxygenfunction:: ouster::sensor::get_lidar_port

.. doxygenfunction:: ouster::sensor::get_imu_port

.. doxygenfunction:: ouster::sensor::poll_client

.. doxygenfunction:: ouster::sensor::read_lidar_packet
      
.. doxygenfunction:: ouster::sensor::read_imu_packet

.. doxygenfunction:: ouster::sensor::set_config

.. doxygenfunction:: ouster::sensor::get_config

.. doxygenfunction:: ouster::sensor::parse_config(const std::string &config)
                     
.. doxygenfunction:: ouster::sensor::parse_metadata

.. doxygenfunction:: ouster::sensor::metadata_from_json

Enums
=====

.. doxygenenum:: ouster::sensor::config_flags
