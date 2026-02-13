/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/file.h"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

#include "ouster/deprecation.h"
#include "ouster/osf/file.h"
// Nolint this because logger comes in when debugging
#include "ouster/impl/logging.h"  // NOLINT(misc-include-cleaner)
#include "ouster/osf/basics.h"
#include "ouster/osf/crc32.h"
#include "ouster/osf/impl/compat_ops.h"
#include "ouster/osf/impl/fb_utils.h"
#include "ouster/osf/offset.h"
#include "ouster/version.h"

using namespace ouster::sdk::core;

namespace ouster {
namespace sdk {
namespace osf {

namespace {

// Print errors only in DEBUG mode
#ifndef NDEBUG
inline void print_error(const std::string& filename, const std::string& msg) {
    ouster::sdk::core::logger().error("ERROR: Osf[{}]: {}", filename, msg);
}
#else
inline void print_error(const std::string& /*filename*/,
                        const std::string& /*msg*/) {}
#endif

}  // namespace

/// ============ OsfFile ============

OsfFile::OsfFile(const std::string& path)
    : state_(FileState::BAD), path_(path) {}

std::string OsfFile::path() const { return path_; }

OsfOffset OsfFile::metadata_offset() { return metadata_offset_; }

// NOTE - this computes chunk sizes with the assumption that chunks are
// contiguous and continue up to the beginning of the metadata.
OsfOffset OsfFile::chunks_offset() {
    auto temp_offset = header_chunk_.size();
    return {temp_offset, metadata_offset().offset() - temp_offset};
}

static Version deserialized_version(uint64_t integer_version) {
    // this method handles converting from the integer representation
    // which includes values in the OSF_VERSION enum (now considered "legacy"
    // code)
    using ouster::sdk::core::Version;
    switch (OsfVersion(integer_version)) {
        case OsfVersion::V_INVALID:
            throw std::runtime_error("Invalid file version.");
        case OsfVersion::V_1_0:
            return Version{1, 0, 0};
        case OsfVersion::V_1_1:
            return Version{1, 1, 0};
        case OsfVersion::V_1_2:
            return Version{1, 2, 0};
        case OsfVersion::V_1_3:
            return Version{1, 3, 0};
        case OsfVersion::V_1_4:
            return Version{1, 4, 0};
        case OsfVersion::V_2_0:
            return Version{2, 0, 0};
        case OsfVersion::V_2_1:
            return Version{2, 1, 0};
        default:
            uint16_t major = (integer_version >> 48) & 0xffff;
            uint16_t minor = (integer_version >> 32) & 0xffff;
            uint16_t patch = (integer_version >> 16) & 0xffff;
            auto vers = Version{major, minor, patch};
            if (vers < Version{2, 1, 0}) {
                // the version occurs before 2.1.0 but isn't defined in the
                // legacy version enum
                throw std::logic_error("Invalid file version.");
            }
            return vers;
    }
}

OUSTER_DIAGNOSTIC_PUSH
OUSTER_DIAGNOSTIC_IGNORE_DEPRECATED
const ouster::sdk::core::Version OsfFile::current_version{2, 1, 0};
OUSTER_DIAGNOSTIC_POP

const ouster::sdk::core::Version OsfFile::CURRENT_VERSION{2, 1, 0};

ouster::sdk::core::Version OsfFile::version() {
    return deserialized_version(static_cast<uint64_t>(version_));
}

uint64_t OsfFile::serialized_version(Version parsed_version) {
    // this method handles converting a semver version back to the integer
    // representation including values defined previously in the OSF_VERSION
    // enum
    using ouster::sdk::core::Version;
    if (parsed_version == Version{1, 0, 0}) {
        return static_cast<uint64_t>(OsfVersion::V_1_0);
    } else if (parsed_version == Version{1, 1, 0}) {
        return static_cast<uint64_t>(OsfVersion::V_1_1);
    } else if (parsed_version == Version{1, 2, 0}) {
        return static_cast<uint64_t>(OsfVersion::V_1_2);
    } else if (parsed_version == Version{1, 3, 0}) {
        return static_cast<uint64_t>(OsfVersion::V_1_3);
    } else if (parsed_version == Version{1, 4, 0}) {
        return static_cast<uint64_t>(OsfVersion::V_1_4);
    } else if (parsed_version == Version{2, 0, 0}) {
        return static_cast<uint64_t>(OsfVersion::V_2_0);
    } else if (parsed_version == Version{2, 1, 0}) {
        return static_cast<uint64_t>(OsfVersion::V_2_1);
    } else if (parsed_version < Version{2, 1, 0}) {
        // the version occurs before 2.1.0 but isn't defined in the legacy
        // version enum
        throw std::logic_error("Invalid file version.");
    }
    return ((static_cast<uint64_t>(parsed_version.major) << 48) |
            (static_cast<uint64_t>(parsed_version.minor) << 32) |
            (static_cast<uint64_t>(parsed_version.patch) << 16));
}

bool OsfFile::good() const {
    bool result = true;
    if (state_ == FileState::BAD) {
        print_error(path_, "Filestate is bad");
        result = false;
    }
    return result;
}

bool OsfFile::operator!() const { return !good(); };

OsfFile::operator bool() const { return good(); };

// ======= Helpers =============

void OsfFile::error(const std::string& msg) {
    state_ = FileState::BAD;
    if (!msg.empty()) {
        print_error(path_, msg);
    } else {
        print_error(path_, get_last_error());
    }
}

std::string OsfFile::to_string() {
    // TODO[tws] an OSF that has been moved won't have a buffer, but the version
    // is derived from this, technically
    std::string version_string;
    try {
        version_string = version().simple_version_string();
    } catch (const std::runtime_error&) {
        version_string = "INVALID";
    }
    std::stringstream string_stream;
    string_stream << "OsfFile [path = '" << path_ << "', "
                  << "state = " << static_cast<int>(state_) << ", "
                  << "version = " << version_string << ", ";
    if (this->good()) {
        auto osf_header = impl::get_osf_header_from_buf(header_chunk_.data());
        string_stream << ", osf.file_length = " << osf_header->file_length()
                      << ", "
                      << "osf.metadata_offset = "
                      << osf_header->metadata_offset() << ", "
                      << "osf.status = "
                      << static_cast<int>(osf_header->status());
    }
    string_stream << "]";
    return string_stream.str();
}

// ======= Move semantics ===================

OsfFile::OsfFile(OsfFile&& other)
    : state_(other.state_),
      path_(other.path_),
      version_(other.version_),
      header_chunk_(std::move(other.header_chunk_)),
      metadata_chunk_(std::move(other.metadata_chunk_)),
      header_offset_(std::move(other.header_offset_)),
      metadata_offset_(std::move(other.metadata_offset_)) {
    other.state_ = FileState::BAD;
    other.version_ = OsfVersion::V_INVALID;
}

OsfFile& OsfFile::operator=(OsfFile&& other) {
    if (this != &other) {
        path_ = other.path_;
        state_ = other.state_;
        version_ = other.version_;
        header_chunk_ = std::move(other.header_chunk_);
        metadata_chunk_ = std::move(other.metadata_chunk_);
        header_offset_ = std::move(other.header_offset_);
        metadata_offset_ = std::move(other.metadata_offset_);
        other.state_ = FileState::BAD;
        other.version_ = OsfVersion::V_INVALID;
    }
    return *this;
}

// ========= Release resources =================

void OsfFile::close() { state_ = FileState::BAD; }

OsfFile::~OsfFile() = default;

void OsfFile::initialize_header_and_metadata() {
    // TODO[tws] consider doing the following in the OsfFile initializer list,
    // since doing so would obviate the need for OsfOffset::valid
    state_ = FileState::GOOD;

    OsfOffset prefix_sized_offset{0, 4};
    OsfBuffer prefix_sized_buf = read(prefix_sized_offset);
    const uint32_t header_size = get_prefixed_size(prefix_sized_buf);

    OsfOffset header_offset{
        prefix_sized_offset.offset(),
        FLATBUFFERS_PREFIX_LENGTH + header_size + osf::CRC_BYTES_SIZE};
    header_offset_ = header_offset;
    header_chunk_ = read(header_offset);

    // Check flatbuffers osfHeader validity
    if (!impl::verify_osf_header_buf(
            header_chunk_.data(), header_chunk_.size() - osf::CRC_BYTES_SIZE)) {
        throw std::runtime_error("OSF header verification has failed.");
    }

    if (!check_prefixed_size_block_crc(header_chunk_, header_chunk_.size())) {
        throw std::runtime_error("OSF header has an invalid CRC.");
    }

    // Get the parsed header and check its contents
    auto osf_header = impl::get_osf_header_from_buf(header_chunk_.data());

    version_ = static_cast<OsfVersion>(osf_header->version());
    if (version_ < OsfVersion::V_2_0) {
        // Check flatbuffers osfSession validity [V1]
        throw std::runtime_error("OSF prior version 2.0 is not supported!");
    }

    if (osf_header->status() !=
        v2::HEADER_STATUS::VALID) {  // NOLINT(misc-include-cleaner)
        ouster::sdk::core::logger().warn(
            "Osf: File metadata not marked as valid in header.");
        return;
    }

    // check that we have enough space for the metadata
    auto metadata_size =
        osf_header->file_length() - osf_header->metadata_offset();
    if (osf_header->metadata_offset() + metadata_size > size()) {
        ouster::sdk::core::logger().warn(
            "Osf: Not enough space in file for metadata.");
        return;
    }

    // Get the metadata offset and chunk
    OsfOffset metadata_offset{osf_header->metadata_offset(), metadata_size};
    metadata_offset_ = metadata_offset;
    metadata_chunk_ = read(metadata_offset);

    // Check flatbuffers metadata validity
    if (!ouster::sdk::osf::impl::check_osf_metadata_buf(metadata_chunk_)) {
        metadata_chunk_ = {};
        ouster::sdk::core::logger().warn(
            "Osf: Metadata verification has failed.");
        return;
    }
}

OsfOffset OsfFile::get_header_chunk_offset() { return header_offset_; }

OsfOffset OsfFile::get_metadata_chunk_offset() { return metadata_offset_; }

const OsfBuffer& OsfFile::get_header_chunk() { return header_chunk_; }

const OsfBuffer& OsfFile::get_metadata_chunk() { return metadata_chunk_; }

}  // namespace osf
}  // namespace sdk
}  // namespace ouster

std::size_t std::hash<ouster::sdk::osf::OsfOffset>::operator()(
    const ouster::sdk::osf::OsfOffset& key) const {
    return std::hash<uint64_t>()(key.offset()) ^
           std::hash<uint64_t>()(key.size());
}
