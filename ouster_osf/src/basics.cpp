/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/basics.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "nonstd/optional.hpp"
#include "ouster/osf/crc32.h"

namespace ouster {
namespace osf {

using nonstd::make_optional;
using nonstd::nullopt;
using nonstd::optional;

namespace impl {

// TODO[pb]: Review some time later these enum/strings converters ...
// Copying all functions to handling enums from ouster-example
// later we will not copy and share code in some other way

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

extern const Table<ChunksLayout, const char*, 2> chunks_layout_strings{
    {{ChunksLayout::LAYOUT_STANDARD, "STANDARD"},
     {ChunksLayout::LAYOUT_STREAMING, "STREAMING"}}};

}  //  namespace impl

/* String conversion */

template <typename K, typename V, size_t N>
static optional<V> lookup(const impl::Table<K, V, N> table, const K& k) {
    auto end = table.end();
    auto res = std::find_if(table.begin(), end, [&](const std::pair<K, V>& p) {
        return p.first == k;
    });

    return res == end ? nullopt : make_optional<V>(res->second);
}

template <typename K, size_t N>
static optional<K> rlookup(const impl::Table<K, const char*, N> table,
                           const char* v) {
    auto end = table.end();
    auto res = std::find_if(table.begin(), end,
                            [&](const std::pair<K, const char*>& p) {
                                return std::strcmp(p.second, v) == 0;
                            });

    return res == end ? nullopt : make_optional<K>(res->first);
}

std::string to_string(ChunksLayout chunks_layout) {
    auto res = lookup(impl::chunks_layout_strings, chunks_layout);
    return res ? res.value() : "UNKNOWN";
}

ChunksLayout chunks_layout_of_string(const std::string& s) {
    auto res = rlookup(impl::chunks_layout_strings, s.c_str());
    return res ? res.value() : ChunksLayout::LAYOUT_STANDARD;
}

std::string to_string(const HEADER_STATUS status) {
    return v2::EnumNamesHEADER_STATUS()[static_cast<int>(status)];
}

std::string to_string(const uint8_t* buf, const size_t count,
                      const size_t max_show_count) {
    std::stringstream ss;
    ss << std::hex;
    size_t show_count = count;
    if (max_show_count != 0 && max_show_count < count)
        show_count = max_show_count;
    for (size_t i = 0; i < show_count; ++i) {
        if (i > 0) ss << " ";
        ss << std::setfill('0') << std::setw(2) << static_cast<int>(buf[i]);
    }
    if (show_count < count) {
        ss << " ... and " << std::dec << (count - show_count) << " more ...";
    }
    return ss.str();
}

std::string read_text_file(const std::string& filename) {
    std::stringstream buf{};
    std::ifstream ifs{};
    ifs.open(filename);
    buf << ifs.rdbuf();
    ifs.close();

    if (!ifs) {
        std::stringstream ss;
        ss << "Failed to read file: " << filename;
        throw std::runtime_error(ss.str());
    }

    return buf.str();
}

uint32_t get_prefixed_size(const uint8_t* buf) {
    return buf[0] + (buf[1] << 8u) + (buf[2] << 16u) + (buf[3] << 24u);
}

uint32_t get_block_size(const uint8_t* buf) {
    return get_prefixed_size(buf) + FLATBUFFERS_PREFIX_LENGTH + CRC_BYTES_SIZE;
}

bool check_prefixed_size_block_crc(const uint8_t* buf,
                                   const uint32_t buf_length) {
    uint32_t prefixed_size = get_prefixed_size(buf);
    if (buf_length < prefixed_size + FLATBUFFERS_PREFIX_LENGTH +
                         ouster::osf::CRC_BYTES_SIZE) {
        std::cerr << "ERROR: CRC32 validation failed!"
                  << " (out of buffer legth)" << std::endl;
        return false;
    }

    const uint32_t crc_stored =
        get_prefixed_size(buf + prefixed_size + FLATBUFFERS_PREFIX_LENGTH);
    const uint32_t crc_calculated =
        osf::crc32(buf, prefixed_size + FLATBUFFERS_PREFIX_LENGTH);

    const bool res = (crc_stored == crc_calculated);

    if (!res) {
        std::cerr << "ERROR: CRC32 validation failed!" << std::endl;
        std::cerr << std::hex << "  CRC -     stored: " << crc_stored
                  << std::dec << std::endl;
        std::cerr << std::hex << "  CRC - calculated: " << crc_calculated
                  << std::dec << std::endl;
    }
    return res;
}

std::function<void(const osf::ts_t, const uint8_t*)> make_build_ls(
    const ouster::sensor::sensor_info& info,
    const LidarScanFieldTypes& ls_field_types,
    std::function<void(const ts_t, const ouster::LidarScan&)> handler) {
    const auto w = info.format.columns_per_frame;
    const auto h = info.format.pixels_per_column;
    auto temp_ls_field_types = ls_field_types;
    std::shared_ptr<LidarScan> ls(nullptr);
    if (temp_ls_field_types.empty()) {
        temp_ls_field_types = get_field_types(info);
    }
    ls = std::make_shared<LidarScan>(w, h, temp_ls_field_types.begin(),
                                     temp_ls_field_types.end());

    auto pf = ouster::sensor::get_format(info);
    auto build_ls_imp = ScanBatcher(w, pf);
    osf::ts_t first_msg_ts{-1};
    return [handler, build_ls_imp, ls, first_msg_ts](
               const osf::ts_t msg_ts, const uint8_t* buf) mutable {
        if (first_msg_ts == osf::ts_t{-1}) {
            first_msg_ts = msg_ts;
        }
        if (build_ls_imp(buf, *ls)) {
            handler(first_msg_ts, *ls);
            // At this point we've just started accumulating new LidarScan, so
            // we are saving the msg_ts (i.e. timestamp of a UDP packet)
            // which contained the first lidar_packet
            first_msg_ts = msg_ts;
        }
    };
}

std::function<void(const osf::ts_t, const uint8_t*)> make_build_ls(
    const ouster::sensor::sensor_info& info,
    std::function<void(const ts_t, const ouster::LidarScan&)> handler) {
    return make_build_ls(info, {}, handler);
}

}  // namespace osf
}  // namespace ouster
