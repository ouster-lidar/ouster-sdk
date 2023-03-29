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

.. doxygenfunction:: ouster::sensor_utils::record_initialize( const std::string& file, const std::string& src_ip, const std::string& dst_ip, int frag_size, bool use_sll_encapsulation = false)

.. doxygenfunction:: ouster::sensor_utils::record_initialize(const std::string& file, int frag_size, bool use_sll_encapsulation = false);

.. doxygenfunction:: ouster::sensor_utils::record_uninitialize

.. doxygenfunction:: ouster::sensor_utils::record_packet(record_handle& handle, const std::string& src_ip, const std::string& dst_ip, int src_port, int dst_port, const uint8_t* buf, size_t buffer_size, uint64_t microsecond_timestamp)


