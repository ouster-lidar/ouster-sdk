=========
os_pcap.h
=========

.. contents::
    :local:

Packet Info
===========

.. doxygenstruct:: ouster::sensor_utils::packet_info
    :members:

.. doxygenfunction:: ouster::sensor_utils::operator<<(std::ostream& stream_in, const packet_info& data)

Handles
=======

.. doxygenstruct:: ouster::sensor_utils::record_handle

.. doxygenstruct:: ouster::sensor_utils::playback_handle

Functions
=========

.. doxygenfunction:: ouster::sensor_utils::replay_initialize

.. doxygenfunction:: ouster::sensor_utils::replay_uninitialize

.. doxygenfunction:: ouster::sensor_utils::replay_reset

.. doxygenfunction:: ouster::sensor_utils::next_packet_info

.. doxygenfunction:: ouster::sensor_utils::read_packet

.. doxygenfunction:: ouster::sensor_utils::record_initialize

.. doxygenfunction:: ouster::sensor_utils::record_uninitialize

.. doxygenfunction:: ouster::sensor_utils::record_packet


