/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "fb_utils.h"

#include <fstream>
#include <iostream>

#include "ouster/impl/logging.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/crc32.h"

using namespace ouster::sensor;

namespace ouster {
namespace osf {

bool check_osf_metadata_buf(const uint8_t* buf, const uint32_t buf_size) {
    // note: fb verifier checks for exact size of buffer equal to prefixed size
    auto verifier =
        flatbuffers::Verifier(buf, buf_size - ouster::osf::CRC_BYTES_SIZE);
    return check_prefixed_size_block_crc(buf, buf_size) &&
           gen::VerifySizePrefixedMetadataBuffer(verifier);
}

bool check_osf_chunk_buf(const uint8_t* buf, const uint32_t buf_size) {
    // note: fb verifier checks for exact size of buffer equal to prefixed size
    auto verifier =
        flatbuffers::Verifier(buf, buf_size - ouster::osf::CRC_BYTES_SIZE);
    return check_prefixed_size_block_crc(buf, buf_size) &&
           gen::VerifySizePrefixedChunkBuffer(verifier);
}

template <typename T>
std::vector<T> vector_from_fb_vector(const flatbuffers::Vector<T>* fb_vec) {
    if (fb_vec == nullptr) return {};
    return {fb_vec->data(), fb_vec->data() + fb_vec->size()};
}

template std::vector<uint8_t> vector_from_fb_vector(
    const flatbuffers::Vector<uint8_t>* fb_vec);
template std::vector<double> vector_from_fb_vector(
    const flatbuffers::Vector<double>* fb_vec);
template std::vector<int> vector_from_fb_vector(
    const flatbuffers::Vector<int>* fb_vec);

// ============ File operations ==========================

uint64_t buffer_to_file(const uint8_t* buf, const uint64_t size,
                        const std::string& filename, bool append) {
    uint64_t saved_size = 0;

    uint32_t crc_res = osf::crc32(buf, size);

    std::fstream file_stream;
    if (append)
        file_stream.open(filename, std::fstream::out | std::fstream::app |
                                       std::fstream::binary);
    else {
        file_stream.open(filename, std::fstream::out | std::fstream::trunc |
                                       std::fstream::binary);
    }

    if (file_stream.is_open()) {
        file_stream.write(reinterpret_cast<const char*>(buf), size);
        if (!file_stream.good()) return 0;
        file_stream.write(reinterpret_cast<char*>(&crc_res), sizeof(uint32_t));
        if (!file_stream.good()) return 0;
        file_stream.close();
        saved_size = size + 4;
    } else {
        logger().error("ERROR: Failed to open {} for writing", filename);
    }
    return saved_size;
}

uint64_t builder_to_file(flatbuffers::FlatBufferBuilder& builder,
                         const std::string& filename, bool append) {
    // Get buffer and save to file
    const uint8_t* buf = builder.GetBufferPointer();
    uint32_t size = builder.GetSize();
    return buffer_to_file(buf, size, filename, append);
}

uint64_t start_osf_file(const std::string& filename) {
    auto header_fbb = flatbuffers::FlatBufferBuilder(1024);
    auto header = ouster::osf::gen::CreateHeader(
        header_fbb, ouster::osf::OSF_VERSION::V_2_1,
        ouster::osf::HEADER_STATUS::INVALID, 0, 0);
    header_fbb.FinishSizePrefixed(header, ouster::osf::gen::HeaderIdentifier());
    return builder_to_file(header_fbb, filename, false);
}

uint64_t finish_osf_file(const std::string& filename,
                         const uint64_t metadata_offset,
                         const uint32_t metadata_size) {
    auto header_fbb = flatbuffers::FlatBufferBuilder(1024);
    auto header = ouster::osf::gen::CreateHeader(
        header_fbb, ouster::osf::OSF_VERSION::V_2_1,
        ouster::osf::HEADER_STATUS::VALID, metadata_offset,
        metadata_offset + metadata_size);
    header_fbb.FinishSizePrefixed(header, ouster::osf::gen::HeaderIdentifier());

    const uint8_t* buf = header_fbb.GetBufferPointer();
    uint32_t size = header_fbb.GetSize();

    uint32_t crc_res = osf::crc32(buf, size);

    uint64_t saved_size = 0;

    std::ofstream file_stream;
    // TODO[pb]: Need to check that file exists here and it contains the OSF
    // header before overwrite it ...
    file_stream.open(filename, std::fstream::out | std::fstream::in |
                                   std::fstream::ate | std::fstream::binary);
    if (file_stream.is_open()) {
        file_stream.seekp(0);

        file_stream.write(reinterpret_cast<const char*>(buf), size);
        // TODO[pb]: It's an exception here .... add error processing
        if (!file_stream.good()) return saved_size;
        saved_size += size;

        file_stream.write(reinterpret_cast<char*>(&crc_res), sizeof(uint32_t));
        if (!file_stream.good()) return saved_size;
        saved_size += sizeof(uint32_t);

        file_stream.close();
    } else {
        logger().error("ERROR: Failed to open {} for writing", filename);
    }

    return saved_size;
}

}  // namespace osf
}  // namespace ouster
