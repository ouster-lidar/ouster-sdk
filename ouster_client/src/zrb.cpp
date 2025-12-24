#include "ouster/zrb.h"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

#include "lsp/zm_defines.h"
#include "ouster/sha256.h"
#include "ouster/vector_streambuf.h"
#include "zone_header.h"

namespace ouster {
namespace sdk {
namespace core {

Zrb::Zrb() : stl_hash(nonstd::nullopt), m_per_zmbin_(DEFAULT_M_PER_ZMBIN) {}

Zrb::Zrb(std::istream& input_stream) : stl_hash(nonstd::nullopt) {
    std::vector<std::string> mismatches;
    if (!read_zone_cache(input_stream, true, mismatches)) {
        std::stringstream err;
        err << "Zrb read error";
        if (!mismatches.empty()) {
            err << ": ";
            for (size_t i = 0; i < mismatches.size(); i++) {
                err << mismatches[i];
                if (i < mismatches.size() - 1) {
                    err << ", ";
                }
            }
        }
        throw std::runtime_error(err.str());
    }
}

Zrb::Zrb(const std::string& file_path) : stl_hash(nonstd::nullopt) {
    std::ifstream in(file_path, std::ios::in | std::ios::binary);
    *this = Zrb(in);
}

Zrb::Zrb(const std::vector<uint8_t>& blob) {
    VectorStreamBuf sbuf(&blob);
    std::istream in(&sbuf);
    *this = Zrb(in);
}

Zrb::Zrb(uint32_t n_rows, uint32_t n_cols, float m_per_zmbin,
         uint64_t serial_number_init, Sha256 stl_hash_init,
         const mat4d& beam_to_lidar_init, const mat4d& lidar_to_sensor_init,
         const mat4d& sensor_to_body_init)
    : near_range_mm{img_t<uint32_t>(n_rows, n_cols)},
      far_range_mm{img_t<uint32_t>(n_rows, n_cols)},
      stl_hash(stl_hash_init),
      beam_to_lidar_transform(beam_to_lidar_init),
      lidar_to_sensor_transform(lidar_to_sensor_init),
      sensor_to_body_transform(sensor_to_body_init),
      serial_number(serial_number_init),
      m_per_zmbin_(m_per_zmbin) {}

std::vector<uint8_t> Zrb::blob() const {
    std::vector<uint8_t> blob;
    VectorStreamBuf sbuf(&blob);
    std::ostream out(&sbuf);
    save(out);
    return blob;
}

void Zrb::save(std::ostream& out) const {
    if (!out) {
        throw std::logic_error("Zrb::save: output stream not valid");
    }
    if (serial_number == 0) {
        throw std::logic_error("Zrb::save: serial number not set");
    }
    if (near_range_mm.size() == 0) {
        throw std::logic_error("Zrb::save: near image data missing");
    }
    if (far_range_mm.size() == 0) {
        throw std::logic_error("Zrb::save: far image data missing");
    }
    if (near_range_mm.cols() != far_range_mm.cols() ||
        near_range_mm.rows() != far_range_mm.rows()) {
        throw std::logic_error(
            "Zrb::save: near_range_mm and far_range_mm images have "
            "different sizes");
    }
    if (beam_to_lidar_transform.isZero() ||
        lidar_to_sensor_transform.isZero() ||
        sensor_to_body_transform.isZero()) {
        throw std::logic_error("Zrb::save: transforms not set for zrb");
    }
    auto n_cols = near_range_mm.cols();
    auto n_rows = near_range_mm.rows();
    uint32_t zr_filesize = n_cols * n_rows;
    std::vector<uint32_t> zr_buffer(zr_filesize, 0);
    float mm_per_bin = m_per_zmbin_ * 1000.f;
    std::bitset<2048> valid_cols;
    for (uint32_t col_id = 0; col_id < n_cols; col_id++) {
        for (uint32_t row_id = 0; row_id < n_rows; row_id++) {
            // NOLINTBEGIN(cppcoreguidelines-narrowing-conversions)
            auto near_range_fp =
                std::round(near_range_mm(row_id, col_id) / mm_per_bin);
            auto far_range_fp =
                std::round(far_range_mm(row_id, col_id) / mm_per_bin);
            if (near_range_fp > UINT16_MAX || far_range_fp > UINT16_MAX) {
                throw std::logic_error(
                    "Zrb::save: range value exceeds maximum encodable "
                    "distance");
            }
            uint32_t near_bins = static_cast<uint32_t>(near_range_fp);
            uint32_t far_bins = static_cast<uint32_t>(far_range_fp);
            // NOLINTEND(cppcoreguidelines-narrowing-conversions)
            uint32_t near_far = (far_bins << 16) | near_bins;
            zr_buffer[(col_id * n_rows) + row_id] = near_far;
            if (far_bins > 0) {
                valid_cols.set(col_id);
            }
        }
    }

    CacheHeader header(*this, valid_cols,
                       serial_number_from_int(serial_number));

    // Calculate bounds hash
    auto bounds_hash = Sha256::hash_blobs(
        {{zr_buffer.data(), zr_buffer.size() * sizeof(uint32_t)}});
    header.meta.bounds_hash = bounds_hash;

    // Calculate total hash (excludes transforms)
    constexpr size_t metadata_hash_size =
        offsetof(CacheRenderMetadata, beam_to_lidar);
    auto total_hash = Sha256::hash_blobs({{&header.meta, metadata_hash_size}});
    header.info.hash = total_hash;

    out.write(reinterpret_cast<const char*>(&header), sizeof(header));
    out.write(
        reinterpret_cast<const char*>(zr_buffer.data()),
        static_cast<std::streamsize>(zr_buffer.size() * sizeof(uint32_t)));
    if (!out) {
        throw std::logic_error("Zrb::save: write failure");
    }
}

void Zrb::save(const std::string& file_path) const {
    std::ofstream file(file_path, std::ios::out | std::ios::binary);
    save(file);
}

Sha256 Zrb::hash() const {
    // TODO[tws] compute the hash when we compute the blob,
    // cache it, and return the cached value here.
    // This prevents recomputing the blob if the API user
    // calls hash() and then blob() or save().
    // Or, better yet, have blob return both the blob and the hash.
    auto blob = this->blob();
    return Sha256::hash_blobs({{blob.data(), blob.size()}});
};

bool operator==(const Zrb& lhs, const Zrb& rhs) {
    return (lhs.near_range_mm == rhs.near_range_mm).all() &&
           (lhs.far_range_mm == rhs.far_range_mm).all() &&
           lhs.m_per_zmbin_ == rhs.m_per_zmbin_ &&
           lhs.stl_hash == rhs.stl_hash &&
           lhs.serial_number == rhs.serial_number &&
           lhs.beam_to_lidar_transform == rhs.beam_to_lidar_transform &&
           lhs.lidar_to_sensor_transform == rhs.lidar_to_sensor_transform &&
           lhs.sensor_to_body_transform == rhs.sensor_to_body_transform;
}

bool operator!=(const Zrb& lhs, const Zrb& rhs) { return !(lhs == rhs); }

bool Zrb::read_zone_cache(std::istream& file, bool load_bounds,
                          std::vector<std::string>& mismatches) {
    if (!file) {
        mismatches.emplace_back("could not open file");
        return false;
    }

    // Read config values from zone cache:
    CacheHeader header;
    if (!file.read(reinterpret_cast<char*>(&header), sizeof(header))) {
        mismatches.emplace_back("header");
        return false;
    }

    bool valid = true;
    if (header.info.version != CacheHeaderInfo::ZONE_CACHE_VERSION) {
        mismatches.emplace_back("cache-version");
        valid = false;
    }

    // Calculate and compare bounds hashes
    auto bounds_hash = Sha256::hash_file({file, sizeof(CacheHeader)});
    if (bounds_hash != header.meta.bounds_hash) {
        std::cerr << "ZoneCache: Bounds hash mismatch: " << bounds_hash.str()
                  << " vs " << header.meta.bounds_hash.str() << "\n";
        mismatches.emplace_back("ZRB");
        valid = false;
    }

    // Calculate total hash (excludes transforms)
    constexpr size_t metadata_hash_size =
        offsetof(CacheRenderMetadata, beam_to_lidar);
    auto hash = Sha256::hash_blobs({{&header.meta, metadata_hash_size}});
    if (hash != header.info.hash) {
        std::cerr << "ZoneCache: Hash mismatch: " << hash.str() << " vs "
                  << header.info.hash.str() << "\n";
        mismatches.emplace_back("hash");
        valid = false;
    }

    if (!valid) {
        return false;
    }

    // Load header information
    m_per_zmbin_ = header.meta.m_per_zmbin;
    if (header.meta.stl_hash != Sha256()) {
        stl_hash = header.meta.stl_hash;
    } else {
        stl_hash = nonstd::nullopt;
    }
    auto serial_number_str = std::string(header.meta.serial_number.data(),
                                         header.meta.serial_number.size());
    serial_number = std::stoll(serial_number_str);
    valid_col_mask = header.meta.valid_col_mask;

    // Load transforms from header
    for (int i = 0; i < 16; ++i) {
        beam_to_lidar_transform.data()[i] =
            static_cast<double>(header.meta.beam_to_lidar[i]);
        lidar_to_sensor_transform.data()[i] =
            static_cast<double>(header.meta.lidar_to_sensor[i]);
        sensor_to_body_transform.data()[i] =
            static_cast<double>(header.meta.sensor_to_body[i]);
    }

    if (load_bounds) {
        // Seek to position after header before reading zone map data
        file.seekg(sizeof(CacheHeader));
        // Initialize near/far images
        near_range_mm = img_t<uint32_t>(header.meta.n_rows, header.meta.n_cols);
        far_range_mm = img_t<uint32_t>(header.meta.n_rows, header.meta.n_cols);
        // Read zone map data into near/far images
        float mm_per_bin = m_per_zmbin_ * 1000.f;
        for (uint32_t col_id = 0; col_id < header.meta.n_cols; col_id++) {
            std::array<uint32_t, C_ZONE_MON_MAX_N_ROWS> col_buffer{};
            if (!file.read(reinterpret_cast<char*>(col_buffer.data()),
                           static_cast<std::streamsize>(sizeof(uint32_t) *
                                                        header.meta.n_rows))) {
                std::cerr << "ZoneCache: Cache invalid: file malformed. "
                             "Requires render.\n";
                return false;
            }
            // The near bits are the lower 16 bits, the far bits the upper 16
            for (uint32_t row_id = 0; row_id < header.meta.n_rows; row_id++) {
                uint32_t near_far = col_buffer.at(row_id);
                uint32_t near_bins = near_far & 0xFFFF;
                uint32_t far_bins = near_far >> 16;
                // NOLINTBEGIN(cppcoreguidelines-narrowing-conversions)
                near_range_mm(row_id, col_id) =
                    std::round(near_bins * mm_per_bin);
                far_range_mm(row_id, col_id) =
                    std::round(far_bins * mm_per_bin);
                // NOLINTEND(cppcoreguidelines-narrowing-conversions)
            }
        }
    }
    return true;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
