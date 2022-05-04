========
client.h
========

.. contents::
    :local:

Client Initialization
=====================

.. doxygengroup:: ouster_client_init
    :content-only:

Data Fetching
=============

.. doxygenfunction:: ouster::sensor::poll_client

.. doxygenfunction:: ouster::sensor::read_lidar_packet
      
.. doxygenfunction:: ouster::sensor::read_imu_packet

Config And Metadata
===================

.. doxygenfunction:: ouster::sensor::get_metadata

.. doxygenfunction:: ouster::sensor::get_config

.. doxygenfunction:: ouster::sensor::set_config

.. doxygenenum:: ouster::sensor::config_flags



Network Operations
==================

.. doxygenfunction:: ouster::sensor::get_lidar_port

.. doxygenfunction:: ouster::sensor::get_imu_port

.. doxygenenum:: ouster::sensor::client_state

