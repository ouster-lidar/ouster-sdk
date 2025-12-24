/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/impl/fb_utils.h"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "ouster/impl/logging.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/crc32.h"
#include "ouster/osf/file.h"

namespace ouster {
namespace sdk {
namespace osf {
namespace impl {

bool check_osf_metadata_buf(const ouster::sdk::osf::OsfBuffer& buf) {
    // note: fb verifier checks for exact size of buffer equal to prefixed size
    auto verifier = flatbuffers::Verifier(
        buf.data(), buf.size() - ouster::sdk::osf::CRC_BYTES_SIZE);
    return check_prefixed_size_block_crc(buf, buf.size()) &&
           gen::VerifySizePrefixedMetadataBuffer(
               verifier);  // NOLINT(misc-include-cleaner)
}

bool check_osf_chunk_buf(const ouster::sdk::osf::OsfBuffer& buf) {
    // note: fb verifier checks for exact size of buffer equal to prefixed size
    auto verifier = flatbuffers::Verifier(
        buf.data(), buf.size() - ouster::sdk::osf::CRC_BYTES_SIZE);
    return check_prefixed_size_block_crc(buf, buf.size()) &&
           gen::VerifySizePrefixedChunkBuffer(
               verifier);  // NOLINT(misc-include-cleaner)
}

template <typename T>
std::vector<T> vector_from_fb_vector(const flatbuffers::Vector<T>* fb_vec) {
    if (fb_vec == nullptr) {
        return {};
    }
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
    if (append) {
        file_stream.open(filename, std::fstream::out | std::fstream::app |
                                       std::fstream::binary);
    } else {
        file_stream.open(filename, std::fstream::out | std::fstream::trunc |
                                       std::fstream::binary);
    }

    if (file_stream.is_open()) {
        file_stream.write(reinterpret_cast<const char*>(buf),
                          static_cast<int64_t>(size));
        if (!file_stream.good()) {
            return 0;
        }
        file_stream.write(reinterpret_cast<char*>(&crc_res), sizeof(uint32_t));
        if (!file_stream.good()) {
            return 0;
        }
        file_stream.close();
        saved_size = size + 4;
    } else {
        ouster::sdk::core::logger().error(
            "ERROR: Failed to open {} for writing", filename);
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
    // NOLINTBEGIN(misc-include-cleaner)
    auto header_fbb = flatbuffers::FlatBufferBuilder(1024);
    auto header = ouster::sdk::osf::impl::gen::CreateHeader(
        header_fbb,
        static_cast<uint64_t>(OsfFile::serialized_version(
            ouster::sdk::osf::OsfFile::CURRENT_VERSION)),
        ouster::sdk::osf::impl::HEADER_STATUS::INVALID, 0, 0);
    header_fbb.FinishSizePrefixed(
        header, ouster::sdk::osf::impl::gen::HeaderIdentifier());
    return builder_to_file(header_fbb, filename, false);
    // NOLINTEND(misc-include-cleaner)
}

uint64_t finish_osf_file(const std::string& filename,
                         const uint64_t metadata_offset,
                         const uint32_t metadata_size,
                         ouster::sdk::core::Version version) {
    auto header_fbb = flatbuffers::FlatBufferBuilder(1024);
    auto header = ouster::sdk::osf::impl::gen::CreateHeader(
        header_fbb, static_cast<uint64_t>(OsfFile::serialized_version(version)),
        ouster::sdk::osf::impl::HEADER_STATUS::VALID, metadata_offset,
        metadata_offset + metadata_size);
    header_fbb.FinishSizePrefixed(
        header, ouster::sdk::osf::impl::gen::HeaderIdentifier());

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
        if (!file_stream.good()) {
            return saved_size;
        }
        saved_size += size;

        file_stream.write(reinterpret_cast<char*>(&crc_res), sizeof(uint32_t));
        if (!file_stream.good()) {
            return saved_size;
        }
        saved_size += sizeof(uint32_t);

        file_stream.close();
    } else {
        ouster::sdk::core::logger().error(
            "ERROR: Failed to open {} for writing", filename);
    }

    return saved_size;
}

}  // namespace impl
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
