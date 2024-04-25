========
basics.h
========

General
-------
.. doxygentypedef:: ouster::osf::ts_t

.. doxygenenum:: ouster::osf::OSF_VERSION

.. doxygenvariable:: ouster::osf::FLATBUFFERS_PREFIX_LENGTH

.. doxygenfunction:: ouster::osf::to_string(const HEADER_STATUS status)

Buffer Operations
-----------------
.. doxygenfunction:: ouster::osf::get_prefixed_size

.. doxygenfunction:: ouster::osf::get_block_size

.. doxygenfunction:: ouster::osf::check_prefixed_size_block_crc

.. doxygenfunction:: ouster::osf::to_string(const uint8_t* buf, const size_t count, const size_t max_show_count = 0)

Batching
--------
.. doxygengroup:: OsfBatchingFunctions
    :content-only:

ChunksLayout
------------
.. doxygenfunction:: ouster::osf::to_string(ChunksLayout chunks_layout)

.. doxygenfunction:: ouster::osf::chunks_layout_of_string

.. doxygenenum:: ouster::osf::ChunksLayout
