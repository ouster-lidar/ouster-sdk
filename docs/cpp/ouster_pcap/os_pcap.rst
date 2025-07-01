os_pcap.h
=========

.. contents::
   :local:

Typedefs
--------

.. doxygentypedef:: ouster::sensor_utils::ts
   :project: cpp_api

Structs
-------

.. doxygenstruct:: ouster::sensor_utils::guessed_ports
   :project: cpp_api
   :members:

.. doxygenstruct:: ouster::sensor_utils::stream_data
   :project: cpp_api
   :members:

.. doxygenstruct:: ouster::sensor_utils::stream_info
   :project: cpp_api
   :members:

.. doxygenstruct:: ouster::sensor_utils::stream_key
   :project: cpp_api
   :members:

.. doxygenstruct:: playback_handle
   :project: cpp_api
   :members:

.. doxygenstruct:: record_handle
   :project: cpp_api
   :members:

.. doxygenstruct:: std::hash< ouster::sensor_utils::stream_key >
   :project: cpp_api
   :members:


Functions
---------

.. doxygenfunction:: ouster::sensor_utils::operator<<(std::ostream&, const ouster::sensor_utils::packet_info&)
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::operator<<(std::ostream&, const ouster::sensor_utils::stream_key&)
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::operator<<(std::ostream&, const ouster::sensor_utils::stream_data&)
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::operator<<(std::ostream&, const ouster::sensor_utils::stream_info&)
   :project: cpp_api

.. doxygenfunction:: ouster::sensor_utils::replay_initialize
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::replay_uninitialize
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::replay_reset
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::next_packet_info
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::read_packet
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::record_initialize
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::record_uninitialize
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::record_packet(record_handle&, const std::string&, const std::string&, int, int, const uint8_t*, size_t, uint64_t)
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::record_packet(record_handle&, const ouster::sensor_utils::packet_info&, const uint8_t*, size_t)
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::get_stream_info(const std::string&, int)
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::get_stream_info(const std::string&, std::function<void(uint64_t, uint64_t, uint64_t)>, int, int)
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::get_stream_info(PcapReader&, std::function<void(uint64_t, uint64_t, uint64_t)>, int, int)
   :project: cpp_api
.. doxygenfunction:: ouster::sensor_utils::guess_ports
   :project: cpp_api