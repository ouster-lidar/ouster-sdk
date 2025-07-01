packet.h
========

.. contents::
   :local:

Enums
-----

.. doxygenenum:: ouster::sensor::PacketType
   :project: cpp_api

.. doxygenenum:: ouster::sensor::PacketValidationFailure
   :project: cpp_api

Structs
-------

.. doxygenstruct:: ouster::sensor::ImuPacket
   :project: cpp_api
   :members:

.. doxygenstruct:: ouster::sensor::LidarPacket
   :project: cpp_api
   :members:

.. doxygenstruct:: ouster::sensor::Packet
   :project: cpp_api
   :members:


Functions
---------

.. doxygenfunction:: ouster::sensor::validate_packet
   :project: cpp_api


Deprecations
------------

.. doxygenfunction:: ouster::sensor_config parse_config
   :project: cpp_api
   :members:

   .. warning::

      This function is **deprecated**. Please switch to using `parse_and_validate_config`.
