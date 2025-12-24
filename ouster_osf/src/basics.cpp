/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/basics.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

#include "nonstd/optional.hpp"
#include "ouster/osf/buffer.h"
#include "ouster/osf/crc32.h"
#include "ouster/osf/impl/basics.h"
#include "ouster/osf/offset.h"

using nonstd::make_optional;
using nonstd::nullopt;
using nonstd::optional;

namespace ouster {
namespace sdk {
namespace osf {

namespace impl {

// TODO[pb]: Review some time later these enum/strings converters ...
// Copying all functions to handling enums from ouster-example
// later we will not copy and share code in some other way

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

extern const Table<ChunksLayout, const char*, 2> CHUNKS_LAYOUT_STRINGS{
    {{ChunksLayout::STANDARD, "STANDARD"},
     {ChunksLayout::STREAMING, "STREAMING"}}};

std::string to_string(
    const HEADER_STATUS status) {  // NOLINT(misc-include-cleaner)
    return v2::EnumNamesHEADER_STATUS()[static_cast<int>(
        status)];  // NOLINT(misc-include-cleaner)
}
}  // namespace impl

namespace {

/* String conversion */

template <typename K, typename V, size_t N>
optional<V> lookup(const impl::Table<K, V, N> table, const K& key) {
    auto end = table.end();
    auto res = std::find_if(
        table.begin(), end,
        [&](const std::pair<K, V>& pair) { return pair.first == key; });

    return res == end ? nullopt : make_optional<V>(res->second);
}

template <typename K, size_t N>
optional<K> rlookup(const impl::Table<K, const char*, N> table,
                    const char* value) {
    auto end = table.end();
    auto res = std::find_if(table.begin(), end,
                            [&](const std::pair<K, const char*>& pair) {
                                return std::strcmp(pair.second, value) == 0;
                            });

    return res == end ? nullopt : make_optional<K>(res->first);
}

}  // namespace

std::string to_string(ChunksLayout chunks_layout) {
    auto res = lookup(impl::CHUNKS_LAYOUT_STRINGS, chunks_layout);
    return res ? res.value() : "UNKNOWN";
}

ChunksLayout chunks_layout_of_string(const std::string& layout_string) {
    auto res = rlookup(impl::CHUNKS_LAYOUT_STRINGS, layout_string.c_str());
    return res ? res.value() : ChunksLayout::STANDARD;
}

std::string to_string(const uint8_t* buf, const size_t count,
                      const size_t max_show_count) {
    std::stringstream stream;
    stream << std::hex;
    size_t show_count = count;
    if (max_show_count != 0 && max_show_count < count) {
        show_count = max_show_count;
    }
    for (size_t i = 0; i < show_count; ++i) {
        if (i > 0) {
            stream << " ";
        }
        stream << std::setfill('0') << std::setw(2) << static_cast<int>(buf[i]);
    }
    if (show_count < count) {
        stream << " ... and " << std::dec << (count - show_count)
               << " more ...";
    }
    return stream.str();
}

std::string read_text_file(const std::string& filename) {
    std::stringstream buf{};
    std::ifstream ifs{};
    ifs.open(filename);
    buf << ifs.rdbuf();
    ifs.close();

    if (!ifs) {
        std::stringstream error_stream;
        error_stream << "Failed to read file: " << filename;
        throw std::runtime_error(error_stream.str());
    }

    return buf.str();
}

uint32_t get_prefixed_size(const OsfBuffer& buf, OsfOffset offset) {
    uint8_t const* data_ptr = buf.data() + offset.offset();
    if (buf.size() < offset.offset() + sizeof(uint32_t)) {
        throw std::runtime_error(
            "Buffer too small to contain prefixed size at given offset");
    }
    return data_ptr[0] + (data_ptr[1] << 8u) + (data_ptr[2] << 16u) +
           (data_ptr[3] << 24u);
}

uint32_t get_prefixed_size(const OsfBuffer& buf) {
    return get_prefixed_size(buf, {0, buf.size()});
}

uint32_t get_block_size(const OsfBuffer& buf) {
    return get_prefixed_size(buf) + FLATBUFFERS_PREFIX_LENGTH + CRC_BYTES_SIZE;
}

bool check_prefixed_size_block_crc(const OsfBuffer& buf,
                                   const uint32_t buf_length) {
    uint32_t prefixed_size = get_prefixed_size(buf);
    if (buf_length <
        prefixed_size + FLATBUFFERS_PREFIX_LENGTH + CRC_BYTES_SIZE) {
        throw std::runtime_error(
            "CRC32 validation failed!"
            " (Prefix Size " +
            std::to_string(prefixed_size) + "[bytes] Buf Length " +
            std::to_string(buf_length) + "[bytes])");
    }

    const uint32_t crc_stored = get_prefixed_size(
        buf, {prefixed_size + FLATBUFFERS_PREFIX_LENGTH, buf.size()});
    const uint32_t crc_calculated =
        osf::crc32(buf.data(), prefixed_size + FLATBUFFERS_PREFIX_LENGTH);

    return (crc_stored == crc_calculated);
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
