client.h
========

.. contents::
   :local:

Variables
---------

.. doxygenvariable:: ouster::sensor::min_version

Enums
-----

.. doxygenenum:: ouster::sensor::client_state
   :project: cpp_api

.. doxygenenum:: ouster::sensor::config_flags
   :project: cpp_api


Functions
---------

.. doxygenfunction:: ouster::sensor::init_logger
   :project: cpp_api

.. doxygengroup:: ouster_client_init
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::poll_client
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::read_lidar_packet(const client &cli, LidarPacket &packet)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::read_lidar_packet(const client &cli, uint8_t *buf, const packet_format &pf)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::read_lidar_packet(const client &cli, uint8_t *buf, size_t bytes)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::read_imu_packet(const client &cli, uint8_t *buf, const packet_format &pf)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::read_imu_packet(const client &cli, uint8_t *buf, size_t bytes)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::get_metadata
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::get_config
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::set_config
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::get_lidar_port
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::get_imu_port
   :project: cpp_api

.. doxygenfunction:: ouster::sensor::in_multicast
   :project: cpp_api