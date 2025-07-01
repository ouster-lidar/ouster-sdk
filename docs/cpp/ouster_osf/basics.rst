basics.h
========

.. contents::
   :local:


Enums
-----

.. doxygenenum:: ouster::osf::OSF_VERSION
   :project: cpp_api

.. doxygenenum:: ouster::osf::ChunksLayout
   :project: cpp_api

Typedefs
--------

.. doxygentypedef:: ouster::osf::ts_t
   :project: cpp_api

Variables
---------

.. doxygenvariable:: ouster::osf::FLATBUFFERS_PREFIX_LENGTH
   :project: cpp_api

Functions
---------

.. doxygenfunction:: ouster::osf::to_string(ChunksLayout chunks_layout)
   :project: cpp_api

.. doxygenfunction:: ouster::osf::chunks_layout_of_string
   :project: cpp_api

.. doxygenfunction:: ouster::osf::to_string(const HEADER_STATUS status)
   :project: cpp_api

.. doxygenfunction:: ouster::osf::to_string(const uint8_t* buf, const size_t count, const size_t max_show_count)
   :project: cpp_api

.. doxygenfunction:: ouster::osf::read_text_file
   :project: cpp_api

.. doxygenfunction:: ouster::osf::get_prefixed_size
   :project: cpp_api

.. doxygenfunction:: ouster::osf::get_block_size
   :project: cpp_api

.. doxygenfunction:: ouster::osf::check_prefixed_size_block_crc
   :project: cpp_api