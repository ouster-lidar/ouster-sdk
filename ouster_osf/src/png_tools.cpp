/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "png_tools.h"

#include <png.h>

#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>

#include "ouster/lidar_scan.h"

namespace ouster {
namespace osf {

/*
 * Effect of png_set_compression(comp level):
 * - (no png out):     2s, n/a
 * - comp level 1:    39s, 648M  (60% speedup vs default, 10% size increase)
 * - comp level 2:    38s, 643M
 * - comp level 3:    45s, 639M
 * - comp level 4:    48s, 590M  (47% speedup vs default, <1% size increase)
 * - comp level 5:    61s, 589M
 * - libpng default:  98s, 586M
 * - comp level 9:   328s, 580M
 *
 * TODO: investigate other zlib options
 */
static constexpr int PNG_OSF_ZLIB_COMPRESSION_LEVEL = 4;

/**
 * Provides the data reader capabilities from std::vector for png_read IO
 */
struct VectorReader {
    const std::vector<uint8_t>& buffer;
    uint32_t offset;
    explicit VectorReader(const std::vector<uint8_t>& buf)
        : buffer(buf), offset(0) {}
    void read(void* bytes, const uint32_t bytes_len) {
        // Skip safety check and trust libpng?
        if (offset >= buffer.size()) return;
        uint32_t bytes_to_read = bytes_len;
        if (offset + bytes_to_read > buffer.size()) {
            bytes_to_read = buffer.size() - offset;
        }
        std::memcpy(bytes, buffer.data() + offset, bytes_to_read);
        offset += bytes_to_read;
    }
};

/**
 * Error callback that will be fired on libpng errors
 */
void png_osf_error(png_structp png_ptr, png_const_charp msg) {
    std::cout << "ERROR libpng osf: " << msg << std::endl;
    longjmp(png_jmpbuf(png_ptr), 1);
};

/**
 * Custom png_write handler to write data to std::vector buffer
 */
void png_osf_write_data(png_structp png_ptr, png_bytep bytes,
                        png_size_t bytes_len) {
    std::vector<uint8_t>* res_buf =
        reinterpret_cast<std::vector<uint8_t>*>(png_get_io_ptr(png_ptr));
    res_buf->insert(res_buf->end(), reinterpret_cast<uint8_t*>(bytes),
                    reinterpret_cast<uint8_t*>(bytes + bytes_len));
};

/**
 * Custom png_read handler to read data from std::vector (via VectorReader
 * helper)
 */
void png_osf_read_data(png_structp png_ptr, png_bytep bytes,
                       png_size_t bytes_len) {
    VectorReader* vec_read =
        reinterpret_cast<VectorReader*>(png_get_io_ptr(png_ptr));
    vec_read->read(bytes, bytes_len);
};

// void user_read_data(png_structp png_ptr, png_bytep data, png_size_t length);

/**
 * It's needed for custom png IO operations... but I've never seen it's called.
 * And also there are no need to flush writer to std::vector buufer in our case.
 */
void png_osf_flush_data(png_structp){};

/**
 * Common png WRITE init routine, creates and setups png_ptr and png_info_ptr
 */
bool png_osf_write_init(png_structpp png_ptrp, png_infopp png_info_ptrp) {
    *png_ptrp = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr,
                                        png_osf_error, png_osf_error);
    if (!*png_ptrp) {
        std::cout << "ERROR: no png_ptr\n";
        return true;
    }

    *png_info_ptrp = png_create_info_struct(*png_ptrp);
    if (!*png_info_ptrp) {
        std::cout << "ERROR: no png_info_ptr\n";
        png_destroy_write_struct(png_ptrp, nullptr);
        return true;
    }

    return false;  // SUCCESS
}

/**
 * Common png READ init routine, creates and setups png_ptr and png_info_ptr
 */
bool png_osf_read_init(png_structpp png_ptrp, png_infopp png_info_ptrp) {
    *png_ptrp = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr,
                                       png_osf_error, png_osf_error);
    if (!*png_ptrp) {
        std::cout << "ERROR: no png_ptr\n";
        return true;
    }

    *png_info_ptrp = png_create_info_struct(*png_ptrp);
    if (!*png_info_ptrp) {
        std::cout << "ERROR: no png_info_ptr\n";
        png_destroy_read_struct(png_ptrp, nullptr, nullptr);
        return true;
    }

    return false;  // SUCCESS
}

/**
 * Common png write setup routine.
 * Write destination is res_buf of std::vector<uitn8_t> type.
 */
void png_osf_write_start(png_structp png_ptr, png_infop png_info_ptr,
                         ScanChannelData& res_buf, const uint32_t width,
                         const uint32_t height, const int sample_depth,
                         const int color_type) {
    // Use setjmp() on upper level for errors catching
    png_set_write_fn(png_ptr, &res_buf, png_osf_write_data, png_osf_flush_data);

    png_set_compression_level(png_ptr, PNG_OSF_ZLIB_COMPRESSION_LEVEL);

    png_set_IHDR(png_ptr, png_info_ptr, width, height, sample_depth, color_type,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png_ptr, png_info_ptr);
}

// ========== Encode Functions ===================================

ScanData scanEncodeFieldsSingleThread(const LidarScan& lidar_scan,
                                      const std::vector<int>& px_offset,
                                      const LidarScanFieldTypes& field_types) {
    // Prepare scan data of size that fits all field_types we are about to
    // encode
    ScanData fields_data(field_types.size());

    size_t scan_idx = 0;
    for (const auto& f : field_types) {
        fieldEncode(lidar_scan, f, px_offset, fields_data, scan_idx);
        scan_idx += 1;
    }

    return fields_data;
}

ScanData scanEncodeFields(const LidarScan& lidar_scan,
                          const std::vector<int>& px_offset,
                          const LidarScanFieldTypes& field_types) {
    // Prepare scan data of size that fits all field_types we are about to
    // encode
    ScanData fields_data(field_types.size());

    unsigned int con_num = std::thread::hardware_concurrency();
    // looking for at least 4 cores if can't determine
    if (!con_num) con_num = 4;

    const size_t fields_num = field_types.size();
    // Number of fields to pack into a single thread coder
    size_t per_thread_num = (fields_num + con_num - 1) / con_num;
    std::vector<std::thread> coders{};
    size_t scan_idx = 0;
    for (size_t t = 0; t < con_num && t * per_thread_num < fields_num; ++t) {
        // Per every thread we pack the `per_thread_num` field_types encodings
        // job
        const size_t start_idx = t * per_thread_num;
        // Fields list for a thread to encode
        LidarScanFieldTypes thread_fields{};
        // Scan indices for the corresponding fields where result will be stored
        std::vector<size_t> thread_idxs{};
        for (size_t i = 0; i < per_thread_num && i + start_idx < fields_num;
             ++i) {
            thread_fields.push_back(field_types[start_idx + i]);
            thread_idxs.push_back(scan_idx);
            scan_idx += 1;
        }
        // Start an encoder thread with selected fields and corresponding
        // indices list
        coders.emplace_back(std::thread{fieldEncodeMulti, std::cref(lidar_scan),
                                        thread_fields, std::cref(px_offset),
                                        std::ref(fields_data), thread_idxs});
    }

    for (auto& t : coders) t.join();

    return fields_data;
}

template <typename T>
bool encode8bitImage(ScanChannelData& res_buf,
                     const Eigen::Ref<const img_t<T>>& img,
                     const std::vector<int>& px_offset) {
    return encode8bitImage<T>(res_buf, destagger(img, px_offset));
}

template bool encode8bitImage<uint8_t>(ScanChannelData&,
                                       const Eigen::Ref<const img_t<uint8_t>>&,
                                       const std::vector<int>&);
template bool encode8bitImage<uint16_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint16_t>>&,
    const std::vector<int>&);
template bool encode8bitImage<uint32_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint32_t>>&,
    const std::vector<int>&);
template bool encode8bitImage<uint64_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint64_t>>&,
    const std::vector<int>&);

template <typename T>
bool encode8bitImage(ScanChannelData& res_buf,
                     const Eigen::Ref<const img_t<T>>& img) {
    const uint32_t width = static_cast<uint32_t>(img.cols());
    const uint32_t height = static_cast<uint32_t>(img.rows());

    // 8 bit Gray
    const int sample_depth = 8;
    const int color_type = PNG_COLOR_TYPE_GRAY;

    // 8bit Encoding Sizes
    std::vector<uint8_t> row_data(width);  // Gray, 8bit

    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_write_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &png_info_ptr);
        return true;
    }

    png_osf_write_start(png_ptr, png_info_ptr, res_buf, width, height,
                        sample_depth, color_type);

    for (size_t u = 0; u < height; ++u) {
        for (size_t v = 0; v < width; ++v) {
            // 8bit Encoding Logic
            row_data[v] = static_cast<uint8_t>(img(u, v));
        }

        png_write_row(png_ptr,
                      reinterpret_cast<png_const_bytep>(row_data.data()));
    }

    png_write_end(png_ptr, nullptr);

    png_destroy_write_struct(&png_ptr, &png_info_ptr);

    return false;  // SUCCESS
}

template bool encode8bitImage<uint8_t>(ScanChannelData&,
                                       const Eigen::Ref<const img_t<uint8_t>>&);
template bool encode8bitImage<uint16_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint16_t>>&);
template bool encode8bitImage<uint32_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint32_t>>&);
template bool encode8bitImage<uint64_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint64_t>>&);

template <typename T>
bool encode16bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img) {
    const uint32_t width = static_cast<uint32_t>(img.cols());
    const uint32_t height = static_cast<uint32_t>(img.rows());

    // 16 bit Gray
    const int sample_depth = 16;
    const int color_type = PNG_COLOR_TYPE_GRAY;

    // 16bit Encoding Sizes
    std::vector<uint8_t> row_data(width * 2);  // Gray, 16bit

    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_write_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &png_info_ptr);
        return true;
    }

    png_osf_write_start(png_ptr, png_info_ptr, res_buf, width, height,
                        sample_depth, color_type);

    // Needed to transform provided little-endian samples to internal
    // PNG big endian format
    png_set_swap(png_ptr);

    for (size_t u = 0; u < height; ++u) {
        for (size_t v = 0; v < width; ++v) {
            const uint64_t key_val = img(u, v);

            // 16bit Encoding Logic
            row_data[v * 2] = static_cast<uint8_t>(key_val & 0xff);
            row_data[v * 2 + 1] = static_cast<uint8_t>((key_val >> 8u) & 0xff);
        }

        png_write_row(png_ptr,
                      reinterpret_cast<png_const_bytep>(row_data.data()));
    }

    png_write_end(png_ptr, nullptr);

    png_destroy_write_struct(&png_ptr, &png_info_ptr);

    return false;  // SUCCESS
}

template bool encode16bitImage<uint8_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint8_t>>&);
template bool encode16bitImage<uint16_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint16_t>>&);
template bool encode16bitImage<uint32_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint32_t>>&);
template bool encode16bitImage<uint64_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint64_t>>&);

template <typename T>
bool encode16bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img,
                      const std::vector<int>& px_offset) {
    return encode16bitImage<T>(res_buf, destagger(img, px_offset));
}

template bool encode16bitImage<uint8_t>(ScanChannelData&,
                                        const Eigen::Ref<const img_t<uint8_t>>&,
                                        const std::vector<int>&);
template bool encode16bitImage<uint16_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint16_t>>&,
    const std::vector<int>&);
template bool encode16bitImage<uint32_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint32_t>>&,
    const std::vector<int>&);
template bool encode16bitImage<uint64_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint64_t>>&,
    const std::vector<int>&);

template <typename T>
bool encode24bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img,
                      const std::vector<int>& px_offset) {
    return encode24bitImage<T>(res_buf, destagger(img, px_offset));
}

template bool encode24bitImage<uint8_t>(ScanChannelData&,
                                        const Eigen::Ref<const img_t<uint8_t>>&,
                                        const std::vector<int>&);
template bool encode24bitImage<uint16_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint16_t>>&,
    const std::vector<int>&);
template bool encode24bitImage<uint32_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint32_t>>&,
    const std::vector<int>&);
template bool encode24bitImage<uint64_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint64_t>>&,
    const std::vector<int>&);

template <typename T>
bool encode24bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img) {
    const uint32_t width = static_cast<uint32_t>(img.cols());
    const uint32_t height = static_cast<uint32_t>(img.rows());

    // 8bit RGB
    const int sample_depth = 8;
    const int color_type = PNG_COLOR_TYPE_RGB;

    // 24bit Encoding Sizes
    std::vector<uint8_t> row_data(width * 3);  // RGB, 8bit

    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_write_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &png_info_ptr);
        return true;
    }

    png_osf_write_start(png_ptr, png_info_ptr, res_buf, width, height,
                        sample_depth, color_type);

    for (size_t u = 0; u < height; ++u) {
        for (size_t v = 0; v < width; ++v) {
            const uint64_t key_val = img(u, v);

            // 24bit Encoding Logic
            row_data[v * 3 + 0] = static_cast<uint8_t>(key_val & 0xff);
            row_data[v * 3 + 1] = static_cast<uint8_t>((key_val >> 8u) & 0xff);
            row_data[v * 3 + 2] = static_cast<uint8_t>((key_val >> 16u) & 0xff);
        }

        png_write_row(png_ptr,
                      reinterpret_cast<png_const_bytep>(row_data.data()));
    }

    png_write_end(png_ptr, nullptr);

    png_destroy_write_struct(&png_ptr, &png_info_ptr);

    return false;  // SUCCESS
}

template bool encode24bitImage<uint8_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint8_t>>&);
template bool encode24bitImage<uint16_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint16_t>>&);
template bool encode24bitImage<uint32_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint32_t>>&);
template bool encode24bitImage<uint64_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint64_t>>&);

template <typename T>
bool encode32bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img,
                      const std::vector<int>& px_offset) {
    return encode32bitImage<T>(res_buf, destagger(img, px_offset));
}

template bool encode32bitImage<uint8_t>(ScanChannelData&,
                                        const Eigen::Ref<const img_t<uint8_t>>&,
                                        const std::vector<int>&);
template bool encode32bitImage<uint16_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint16_t>>&,
    const std::vector<int>&);
template bool encode32bitImage<uint32_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint32_t>>&,
    const std::vector<int>&);
template bool encode32bitImage<uint64_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint64_t>>&,
    const std::vector<int>&);

template <typename T>
bool encode32bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img) {
    const uint32_t width = static_cast<uint32_t>(img.cols());
    const uint32_t height = static_cast<uint32_t>(img.rows());

    // 8bit RGBA
    const int sample_depth = 8;
    const int color_type = PNG_COLOR_TYPE_RGB_ALPHA;

    // 32bit Encoding Sizes
    std::vector<uint8_t> row_data(width * 4);  // RGBA, 8bit

    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_write_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &png_info_ptr);
        return true;
    }

    png_osf_write_start(png_ptr, png_info_ptr, res_buf, width, height,
                        sample_depth, color_type);

    for (size_t u = 0; u < height; ++u) {
        for (size_t v = 0; v < width; ++v) {
            const uint64_t key_val = img(u, v);

            // 32bit Encoding Logic
            row_data[v * 4 + 0] = static_cast<uint8_t>(key_val & 0xff);
            row_data[v * 4 + 1] = static_cast<uint8_t>((key_val >> 8u) & 0xff);
            row_data[v * 4 + 2] = static_cast<uint8_t>((key_val >> 16u) & 0xff);
            row_data[v * 4 + 3] = static_cast<uint8_t>((key_val >> 24u) & 0xff);
        }

        png_write_row(png_ptr,
                      reinterpret_cast<png_const_bytep>(row_data.data()));
    }

    png_write_end(png_ptr, nullptr);

    png_destroy_write_struct(&png_ptr, &png_info_ptr);

    return false;  // SUCCESS
}

template bool encode32bitImage<uint8_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint8_t>>&);
template bool encode32bitImage<uint16_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint16_t>>&);
template bool encode32bitImage<uint32_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint32_t>>&);
template bool encode32bitImage<uint64_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint64_t>>&);

template <typename T>
bool encode64bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img,
                      const std::vector<int>& px_offset) {
    return encode64bitImage<T>(res_buf, destagger(img, px_offset));
}

template bool encode64bitImage<uint8_t>(ScanChannelData&,
                                        const Eigen::Ref<const img_t<uint8_t>>&,
                                        const std::vector<int>&);
template bool encode64bitImage<uint16_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint16_t>>&,
    const std::vector<int>&);
template bool encode64bitImage<uint32_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint32_t>>&,
    const std::vector<int>&);
template bool encode64bitImage<uint64_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint64_t>>&,
    const std::vector<int>&);

template <typename T>
bool encode64bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img) {
    const uint32_t width = static_cast<uint32_t>(img.cols());
    const uint32_t height = static_cast<uint32_t>(img.rows());

    // 16bit RGBA
    const int sample_depth = 16;
    const int color_type = PNG_COLOR_TYPE_RGB_ALPHA;

    // 64bit Encoding Sizes
    std::vector<uint8_t> row_data(width * 8);  // RGBA, 16bit

    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_write_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &png_info_ptr);
        return true;
    }

    png_osf_write_start(png_ptr, png_info_ptr, res_buf, width, height,
                        sample_depth, color_type);

    // Needed to transform provided little-endian samples to internal
    // PNG big endian format
    png_set_swap(png_ptr);

    for (size_t u = 0; u < height; ++u) {
        for (size_t v = 0; v < width; ++v) {
            const uint64_t key_val = img(u, v);

            // 64bit Encoding Logic
            row_data[v * 8 + 0] = static_cast<uint8_t>(key_val & 0xff);
            row_data[v * 8 + 1] = static_cast<uint8_t>((key_val >> 8u) & 0xff);
            row_data[v * 8 + 2] = static_cast<uint8_t>((key_val >> 16u) & 0xff);
            row_data[v * 8 + 3] = static_cast<uint8_t>((key_val >> 24u) & 0xff);
            row_data[v * 8 + 4] = static_cast<uint8_t>((key_val >> 32u) & 0xff);
            row_data[v * 8 + 5] = static_cast<uint8_t>((key_val >> 40u) & 0xff);
            row_data[v * 8 + 6] = static_cast<uint8_t>((key_val >> 48u) & 0xff);
            row_data[v * 8 + 7] = static_cast<uint8_t>((key_val >> 56u) & 0xff);
        }

        png_write_row(png_ptr,
                      reinterpret_cast<png_const_bytep>(row_data.data()));
    }

    png_write_end(png_ptr, nullptr);

    png_destroy_write_struct(&png_ptr, &png_info_ptr);

    return false;  // SUCCESS
}

template bool encode64bitImage<uint8_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint8_t>>&);
template bool encode64bitImage<uint16_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint16_t>>&);
template bool encode64bitImage<uint32_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint32_t>>&);
template bool encode64bitImage<uint64_t>(
    ScanChannelData&, const Eigen::Ref<const img_t<uint64_t>>&);

bool fieldEncodeMulti(const LidarScan& lidar_scan,
                      const LidarScanFieldTypes& field_types,
                      const std::vector<int>& px_offset, ScanData& scan_data,
                      const std::vector<size_t>& scan_idxs) {
    if (field_types.size() != scan_idxs.size()) {
        std::cerr << "ERROR: in fieldEncodeMulti field_types.size() should "
                     "match scan_idxs.size()"
                  << std::endl;
        std::abort();
    }
    auto res_err = false;
    for (size_t i = 0; i < field_types.size(); ++i) {
        auto err = fieldEncode(lidar_scan, field_types[i], px_offset, scan_data,
                               scan_idxs[i]);
        if (err) {
            std::cerr << "ERROR: fieldEncode: Can't encode field ["
                      << sensor::to_string(field_types[i])
                      << "] (in "
                         "fieldEncodeMulti)"
                      << std::endl;
        }
        res_err = res_err || err;
    }
    return res_err;
}

bool fieldEncode(
    const LidarScan& lidar_scan,
    const std::pair<sensor::ChanField, sensor::ChanFieldType> field_type,
    const std::vector<int>& px_offset, ScanData& scan_data, size_t scan_idx) {
    if (scan_idx >= scan_data.size()) {
        std::cerr << "ERROR: scan_data size is not sufficient to hold idx: "
                  << scan_idx << std::endl;
        std::abort();
    }
    bool res = true;
    switch (field_type.second) {
        case sensor::ChanFieldType::UINT8:
            res = encode8bitImage(scan_data[scan_idx],
                                  lidar_scan.field<uint8_t>(field_type.first),
                                  px_offset);
            break;
        case sensor::ChanFieldType::UINT16:
            res = encode16bitImage(scan_data[scan_idx],
                                   lidar_scan.field<uint16_t>(field_type.first),
                                   px_offset);
            break;
        case sensor::ChanFieldType::UINT32:
            res = encode32bitImage(scan_data[scan_idx],
                                   lidar_scan.field<uint32_t>(field_type.first),
                                   px_offset);
            break;
        case sensor::ChanFieldType::UINT64:
            res = encode64bitImage(scan_data[scan_idx],
                                   lidar_scan.field<uint64_t>(field_type.first),
                                   px_offset);
            break;
        default:
            std::cerr << "ERROR: fieldEncode: UNKNOWN: ChanFieldType not yet "
                         "implemented"
                      << std::endl;
            break;
    }
    if (res) {
        std::cerr << "ERROR: fieldEncode: Can't encode field "
                  << sensor::to_string(field_type.first) << std::endl;
    }
    return res;
}

ScanData scanEncode(const LidarScan& lidar_scan,
                    const std::vector<int>& px_offset) {
#ifdef OUSTER_OSF_NO_THREADING
    return scanEncodeFieldsSingleThread(lidar_scan, px_offset,
                                        {lidar_scan.begin(), lidar_scan.end()});
#else
    return scanEncodeFields(lidar_scan, px_offset,
                            {lidar_scan.begin(), lidar_scan.end()});
#endif
}

// ========== Decode Functions ===================================

bool scanDecode(LidarScan& lidar_scan, const ScanData& scan_data,
                const std::vector<int>& px_offset) {
#ifdef OUSTER_OSF_NO_THREADING
    return scanDecodeFieldsSingleThread(lidar_scan, scan_data, px_offset);
#else
    return scanDecodeFields(lidar_scan, scan_data, px_offset);
#endif
}

bool fieldDecode(
    LidarScan& lidar_scan, const ScanData& scan_data, size_t start_idx,
    const std::pair<sensor::ChanField, sensor::ChanFieldType> field_type,
    const std::vector<int>& px_offset) {
    switch (field_type.second) {
        case sensor::ChanFieldType::UINT8:
            return decode8bitImage(lidar_scan.field<uint8_t>(field_type.first),
                                   scan_data[start_idx], px_offset);
            return true;
        case sensor::ChanFieldType::UINT16:
            return decode16bitImage(
                lidar_scan.field<uint16_t>(field_type.first),
                scan_data[start_idx], px_offset);
        case sensor::ChanFieldType::UINT32:
            return decode32bitImage(
                lidar_scan.field<uint32_t>(field_type.first),
                scan_data[start_idx], px_offset);
        case sensor::ChanFieldType::UINT64:
            return decode64bitImage(
                lidar_scan.field<uint64_t>(field_type.first),
                scan_data[start_idx], px_offset);
            return true;
        default:
            std::cout << "ERROR: fieldDecode: UNKNOWN: ChanFieldType not yet "
                         "implemented"
                      << std::endl;
            return true;
    }
    return true;  // ERROR
}

bool fieldDecodeMulti(LidarScan& lidar_scan, const ScanData& scan_data,
                      const std::vector<size_t>& scan_idxs,
                      const LidarScanFieldTypes& field_types,
                      const std::vector<int>& px_offset) {
    if (field_types.size() != scan_idxs.size()) {
        std::cerr << "ERROR: in fieldDecodeMulti field_types.size() should "
                     "match scan_idxs.size()"
                  << std::endl;
        std::abort();
    }
    auto res_err = false;
    for (size_t i = 0; i < field_types.size(); ++i) {
        auto err = fieldDecode(lidar_scan, scan_data, scan_idxs[i],
                               field_types[i], px_offset);
        if (err) {
            std::cerr << "ERROR: fieldDecodeMulti: Can't decode field ["
                      << sensor::to_string(field_types[i]) << "]" << std::endl;
        }
        res_err = res_err || err;
    }
    return res_err;
}

bool scanDecodeFieldsSingleThread(LidarScan& lidar_scan,
                                  const ScanData& scan_data,
                                  const std::vector<int>& px_offset) {
    size_t fields_cnt = std::distance(lidar_scan.begin(), lidar_scan.end());
    if (scan_data.size() != fields_cnt) {
        std::cerr << "ERROR: lidar_scan data contains # of channels: "
                  << scan_data.size() << ", expected: " << fields_cnt
                  << " for OSF_EUDP" << std::endl;
        return true;
    }
    size_t next_idx = 0;
    for (auto f : lidar_scan) {
        if (fieldDecode(lidar_scan, scan_data, next_idx, f, px_offset)) {
            std::cout << "ERROR: scanDecodeFields: Failed to decode field"
                      << std::endl;
            return true;
        }
        ++next_idx;
    }
    return false;
}

bool scanDecodeFields(LidarScan& lidar_scan, const ScanData& scan_data,
                      const std::vector<int>& px_offset) {
    LidarScanFieldTypes field_types(lidar_scan.begin(), lidar_scan.end());
    size_t fields_num = field_types.size();
    if (scan_data.size() != fields_num) {
        std::cerr << "ERROR: lidar_scan data contains # of channels: "
                  << scan_data.size() << ", expected: " << fields_num
                  << " for OSF EUDP" << std::endl;
        return true;
    }

    unsigned int con_num = std::thread::hardware_concurrency();
    // looking for at least 4 cores if can't determine
    if (!con_num) con_num = 4;

    // Number of fields to pack into a single thread coder
    size_t per_thread_num = (fields_num + con_num - 1) / con_num;
    std::vector<std::thread> coders{};
    size_t scan_idx = 0;

    for (size_t t = 0; t < con_num && t * per_thread_num < fields_num; ++t) {
        // Per every thread we pack the `per_thread_num` field_types encodings
        // job
        const size_t start_idx = t * per_thread_num;
        // Fields list for a thread to encode
        LidarScanFieldTypes thread_fields{};
        // Scan indices for the corresponding fields where result will be stored
        std::vector<size_t> thread_idxs{};
        for (size_t i = 0; i < per_thread_num && i + start_idx < fields_num;
             ++i) {
            thread_fields.push_back(field_types[start_idx + i]);
            thread_idxs.push_back(scan_idx);
            scan_idx += 1;  // for UINT64 can be 2 (NOT IMPLEMENTED YET)
        }

        // Start a decoder thread with selected fields and corresponding
        // indices list

        coders.emplace_back(std::thread{fieldDecodeMulti, std::ref(lidar_scan),
                                        std::cref(scan_data), thread_idxs,
                                        thread_fields, std::cref(px_offset)});
    }

    for (auto& t : coders) t.join();

    return false;
}

template <typename T>
bool decode24bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf,
                      const std::vector<int>& px_offset) {
    if (!decode24bitImage<T>(img, channel_buf)) {
        img = stagger<T>(img, px_offset);
        return false;  // SUCCESS
    }
    return true;  // ERROR
}

template bool decode24bitImage<uint8_t>(Eigen::Ref<img_t<uint8_t>>,
                                        const ScanChannelData&,
                                        const std::vector<int>&);
template bool decode24bitImage<uint16_t>(Eigen::Ref<img_t<uint16_t>>,
                                         const ScanChannelData&,
                                         const std::vector<int>&);
template bool decode24bitImage<uint32_t>(Eigen::Ref<img_t<uint32_t>>,
                                         const ScanChannelData&,
                                         const std::vector<int>&);
template bool decode24bitImage<uint64_t>(Eigen::Ref<img_t<uint64_t>>,
                                         const ScanChannelData&,
                                         const std::vector<int>&);

template <typename T>
bool decode24bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf) {
    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_read_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);
        return true;
    }

    VectorReader channel_reader(channel_buf);
    png_set_read_fn(png_ptr, &channel_reader, png_osf_read_data);

    // only used with Gray 16 bit to get little-endian LSB representation back
    // for other color types PNG_TRANSFORM_SWAP_ENDIAN does nothing
    int transforms = PNG_TRANSFORM_SWAP_ENDIAN;
    png_read_png(png_ptr, png_info_ptr, transforms, nullptr);

    png_uint_32 width;
    png_uint_32 height;
    int sample_depth;
    int color_type;

    png_get_IHDR(png_ptr, png_info_ptr, &width, &height, &sample_depth,
                 &color_type, nullptr, nullptr, nullptr);

    png_bytepp row_pointers = png_get_rows(png_ptr, png_info_ptr);

    // Sanity checks for encoded PNG size
    if (width != static_cast<png_uint_32>(img.cols()) ||
        height != static_cast<png_uint_32>(img.rows())) {
        std::cout << "ERROR: img contains data of incompatible size: " << width
                  << "x" << height << ", expected: " << img.cols() << "x"
                  << img.rows() << std::endl;
        return true;
    }

    if (sample_depth != 8) {
        std::cout << "ERROR: encoded img contains data with incompatible "
                     "sample_depth: "
                  << sample_depth << ", expected: 8" << std::endl;
        return true;
    }

    if (color_type != PNG_COLOR_TYPE_RGB) {
        std::cout << "ERROR: encoded img contains data with incompatible "
                     "color type: "
                  << color_type << ", expected: " << PNG_COLOR_TYPE_RGB
                  << std::endl;
        return true;
    }

    // 24bit channel data decoding to LidarScan for key channel_index
    for (size_t u = 0; u < height; u++) {
        for (size_t v = 0; v < width; v++) {
            img(u, v) = static_cast<T>(row_pointers[u][v * 3 + 0]) +
                        (static_cast<T>(row_pointers[u][v * 3 + 1]) << 8u) +
                        (static_cast<T>(row_pointers[u][v * 3 + 2]) << 16u);
        }
    }

    png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);

    return false;  // SUCCESS
}

template bool decode24bitImage<uint8_t>(Eigen::Ref<img_t<uint8_t>>,
                                        const ScanChannelData&);
template bool decode24bitImage<uint16_t>(Eigen::Ref<img_t<uint16_t>>,
                                         const ScanChannelData&);
template bool decode24bitImage<uint32_t>(Eigen::Ref<img_t<uint32_t>>,
                                         const ScanChannelData&);
template bool decode24bitImage<uint64_t>(Eigen::Ref<img_t<uint64_t>>,
                                         const ScanChannelData&);

template <typename T>
bool decode32bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf,
                      const std::vector<int>& px_offset) {
    if (!decode32bitImage<T>(img, channel_buf)) {
        img = stagger<T>(img, px_offset);
        return false;  // SUCCESS
    }
    return true;  // ERROR
}

template bool decode32bitImage<uint8_t>(Eigen::Ref<img_t<uint8_t>>,
                                        const ScanChannelData&,
                                        const std::vector<int>&);
template bool decode32bitImage<uint16_t>(Eigen::Ref<img_t<uint16_t>>,
                                         const ScanChannelData&,
                                         const std::vector<int>&);
template bool decode32bitImage<uint32_t>(Eigen::Ref<img_t<uint32_t>>,
                                         const ScanChannelData&,
                                         const std::vector<int>&);
template bool decode32bitImage<uint64_t>(Eigen::Ref<img_t<uint64_t>>,
                                         const ScanChannelData&,
                                         const std::vector<int>&);

template <typename T>
bool decode32bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf) {
    if (sizeof(T) < 4) {
        std::cerr << "WARNING: Attempt to decode image of bigger pixel size"
                  << std::endl;
    }
    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_read_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);
        return true;
    }

    VectorReader channel_reader(channel_buf);
    png_set_read_fn(png_ptr, &channel_reader, png_osf_read_data);

    // only used with Gray 16 bit to get little-endian LSB representation back
    // for other color types PNG_TRANSFORM_SWAP_ENDIAN does nothing
    int transforms = PNG_TRANSFORM_SWAP_ENDIAN;
    png_read_png(png_ptr, png_info_ptr, transforms, nullptr);

    png_uint_32 width;
    png_uint_32 height;
    int sample_depth;
    int color_type;

    png_get_IHDR(png_ptr, png_info_ptr, &width, &height, &sample_depth,
                 &color_type, nullptr, nullptr, nullptr);

    png_bytepp row_pointers = png_get_rows(png_ptr, png_info_ptr);

    // Sanity checks for encoded PNG size
    if (width != static_cast<png_uint_32>(img.cols()) ||
        height != static_cast<png_uint_32>(img.rows())) {
        std::cout << "ERROR: img contains data of incompatible size: " << width
                  << "x" << height << ", expected: " << img.cols() << "x"
                  << img.rows() << std::endl;
        return true;
    }

    if (sample_depth != 8) {
        std::cout << "ERROR: encoded img contains data with incompatible "
                     "sample_depth: "
                  << sample_depth << ", expected: 8" << std::endl;
        return true;
    }

    if (color_type != PNG_COLOR_TYPE_RGB_ALPHA) {
        std::cout << "ERROR: encoded img contains data with incompatible "
                     "color type: "
                  << color_type << ", expected: " << PNG_COLOR_TYPE_RGB_ALPHA
                  << std::endl;
        return true;
    }

    // 32bit channel data decoding to LidarScan for key channel_index
    for (size_t u = 0; u < height; u++) {
        for (size_t v = 0; v < width; v++) {
            img(u, v) = static_cast<T>(row_pointers[u][v * 4 + 0]) +
                        (static_cast<T>(row_pointers[u][v * 4 + 1]) << 8u) +
                        (static_cast<T>(row_pointers[u][v * 4 + 2]) << 16u) +
                        (static_cast<T>(row_pointers[u][v * 4 + 3]) << 24u);
        }
    }

    png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);

    return false;  // SUCCESS
}

template bool decode32bitImage<uint8_t>(Eigen::Ref<img_t<uint8_t>>,
                                        const ScanChannelData&);
template bool decode32bitImage<uint16_t>(Eigen::Ref<img_t<uint16_t>>,
                                         const ScanChannelData&);
template bool decode32bitImage<uint32_t>(Eigen::Ref<img_t<uint32_t>>,
                                         const ScanChannelData&);
template bool decode32bitImage<uint64_t>(Eigen::Ref<img_t<uint64_t>>,
                                         const ScanChannelData&);

template <typename T>
bool decode64bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf,
                      const std::vector<int>& px_offset) {
    if (!decode64bitImage<T>(img, channel_buf)) {
        img = stagger<T>(img, px_offset);
        return false;  // SUCCESS
    }
    return true;  // ERROR
}

template bool decode64bitImage<uint8_t>(Eigen::Ref<img_t<uint8_t>>,
                                        const ScanChannelData&,
                                        const std::vector<int>&);
template bool decode64bitImage<uint16_t>(Eigen::Ref<img_t<uint16_t>>,
                                         const ScanChannelData&,
                                         const std::vector<int>&);
template bool decode64bitImage<uint32_t>(Eigen::Ref<img_t<uint32_t>>,
                                         const ScanChannelData&,
                                         const std::vector<int>&);
template bool decode64bitImage<uint64_t>(Eigen::Ref<img_t<uint64_t>>,
                                         const ScanChannelData&,
                                         const std::vector<int>&);

template <typename T>
bool decode64bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf) {
    if (sizeof(T) < 8) {
        std::cerr << "WARNING: Attempt to decode image of bigger pixel size"
                  << std::endl;
    }
    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_read_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);
        return true;
    }

    VectorReader channel_reader(channel_buf);
    png_set_read_fn(png_ptr, &channel_reader, png_osf_read_data);

    // only used with Gray 16 bit to get little-endian LSB representation back
    // for other color types PNG_TRANSFORM_SWAP_ENDIAN does nothing
    int transforms = PNG_TRANSFORM_SWAP_ENDIAN;
    png_read_png(png_ptr, png_info_ptr, transforms, nullptr);

    png_uint_32 width;
    png_uint_32 height;
    int sample_depth;
    int color_type;

    png_get_IHDR(png_ptr, png_info_ptr, &width, &height, &sample_depth,
                 &color_type, nullptr, nullptr, nullptr);

    png_bytepp row_pointers = png_get_rows(png_ptr, png_info_ptr);

    // Sanity checks for encoded PNG size
    if (width != static_cast<png_uint_32>(img.cols()) ||
        height != static_cast<png_uint_32>(img.rows())) {
        std::cout << "ERROR: img contains data of incompatible size: " << width
                  << "x" << height << ", expected: " << img.cols() << "x"
                  << img.rows() << std::endl;
        return true;
    }

    if (sample_depth != 16) {
        std::cout << "ERROR: encoded img contains data with incompatible "
                     "sample_depth: "
                  << sample_depth << ", expected: 16" << std::endl;
        return true;
    }

    if (color_type != PNG_COLOR_TYPE_RGB_ALPHA) {
        std::cout << "ERROR: encoded img contains data with incompatible "
                     "color type: "
                  << color_type << ", expected: " << PNG_COLOR_TYPE_RGB_ALPHA
                  << std::endl;
        return true;
    }

    // 64bit channel data decoding to LidarScan for key channel_index
    for (size_t u = 0; u < height; u++) {
        for (size_t v = 0; v < width; v++) {
            uint64_t val =
                static_cast<uint64_t>(row_pointers[u][v * 8 + 0]) +
                (static_cast<uint64_t>(row_pointers[u][v * 8 + 1]) << 8u) +
                (static_cast<uint64_t>(row_pointers[u][v * 8 + 2]) << 16u) +
                (static_cast<uint64_t>(row_pointers[u][v * 8 + 3]) << 24u) +
                (static_cast<uint64_t>(row_pointers[u][v * 8 + 4]) << 32u) +
                (static_cast<uint64_t>(row_pointers[u][v * 8 + 5]) << 40u) +
                (static_cast<uint64_t>(row_pointers[u][v * 8 + 6]) << 48u) +
                (static_cast<uint64_t>(row_pointers[u][v * 8 + 7]) << 56u);
            img(u, v) = static_cast<T>(val);
        }
    }

    png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);

    return false;  // SUCCESS
}

template bool decode64bitImage<uint8_t>(Eigen::Ref<img_t<uint8_t>>,
                                        const ScanChannelData&);
template bool decode64bitImage<uint16_t>(Eigen::Ref<img_t<uint16_t>>,
                                         const ScanChannelData&);
template bool decode64bitImage<uint32_t>(Eigen::Ref<img_t<uint32_t>>,
                                         const ScanChannelData&);
template bool decode64bitImage<uint64_t>(Eigen::Ref<img_t<uint64_t>>,
                                         const ScanChannelData&);

template <typename T>
bool decode16bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf,
                      const std::vector<int>& px_offset) {
    if (!decode16bitImage<T>(img, channel_buf)) {
        img = stagger<T>(img, px_offset);
        return false;  // SUCCESS
    }
    return true;  // ERROR
}

template bool decode16bitImage<uint8_t>(Eigen::Ref<img_t<uint8_t>>,
                                        const ScanChannelData&,
                                        const std::vector<int>&);
template bool decode16bitImage<uint16_t>(Eigen::Ref<img_t<uint16_t>>,
                                         const ScanChannelData&,
                                         const std::vector<int>&);
template bool decode16bitImage<uint32_t>(Eigen::Ref<img_t<uint32_t>>,
                                         const ScanChannelData&,
                                         const std::vector<int>&);
template bool decode16bitImage<uint64_t>(Eigen::Ref<img_t<uint64_t>>,
                                         const ScanChannelData&,
                                         const std::vector<int>&);

template <typename T>
bool decode16bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf) {
    if (sizeof(T) < 2) {
        std::cerr << "WARNING: Attempt to decode image of bigger pixel size"
                  << std::endl;
    }
    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_read_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);
        return true;
    }

    VectorReader channel_reader(channel_buf);
    png_set_read_fn(png_ptr, &channel_reader, png_osf_read_data);

    // only used with Gray 16 bit to get little-endian LSB representation back
    // for other color types PNG_TRANSFORM_SWAP_ENDIAN does nothing
    int transforms = PNG_TRANSFORM_SWAP_ENDIAN;
    png_read_png(png_ptr, png_info_ptr, transforms, nullptr);

    png_uint_32 width;
    png_uint_32 height;
    int sample_depth;
    int color_type;

    png_get_IHDR(png_ptr, png_info_ptr, &width, &height, &sample_depth,
                 &color_type, nullptr, nullptr, nullptr);

    png_bytepp row_pointers = png_get_rows(png_ptr, png_info_ptr);

    // Sanity checks for encoded PNG size
    if (width != static_cast<png_uint_32>(img.cols()) ||
        height != static_cast<png_uint_32>(img.rows())) {
        std::cout << "ERROR: img contains data of incompatible size: " << width
                  << "x" << height << ", expected: " << img.cols() << "x"
                  << img.rows() << std::endl;
        return true;
    }

    if (sample_depth != 16) {
        std::cout << "ERROR: encoded img contains data with incompatible "
                     "sample_depth: "
                  << sample_depth << ", expected: 16" << std::endl;
        return true;
    }

    if (color_type != PNG_COLOR_TYPE_GRAY) {
        std::cout << "ERROR: encoded img contains data with incompatible "
                     "color type: "
                  << color_type << ", expected: " << PNG_COLOR_TYPE_GRAY
                  << std::endl;
        return true;
    }

    // 16bit channel data decoding to LidarScan for key channel_index
    for (size_t u = 0; u < height; u++) {
        for (size_t v = 0; v < width; v++) {
            img(u, v) = static_cast<T>(row_pointers[u][v * 2 + 0]) +
                        (static_cast<T>(row_pointers[u][v * 2 + 1]) << 8u);
        }
    }

    png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);

    return false;  // SUCCESS
}

template bool decode16bitImage<uint8_t>(Eigen::Ref<img_t<uint8_t>>,
                                        const ScanChannelData&);
template bool decode16bitImage<uint16_t>(Eigen::Ref<img_t<uint16_t>>,
                                         const ScanChannelData&);
template bool decode16bitImage<uint32_t>(Eigen::Ref<img_t<uint32_t>>,
                                         const ScanChannelData&);
template bool decode16bitImage<uint64_t>(Eigen::Ref<img_t<uint64_t>>,
                                         const ScanChannelData&);

template <typename T>
bool decode8bitImage(Eigen::Ref<img_t<T>> img,
                     const ScanChannelData& channel_buf,
                     const std::vector<int>& px_offset) {
    if (!decode8bitImage<T>(img, channel_buf)) {
        img = stagger<T>(img, px_offset);
        return false;  // SUCCESS
    }
    return true;  // ERROR
}

template bool decode8bitImage<uint8_t>(Eigen::Ref<img_t<uint8_t>>,
                                       const ScanChannelData&,
                                       const std::vector<int>&);
template bool decode8bitImage<uint16_t>(Eigen::Ref<img_t<uint16_t>>,
                                        const ScanChannelData&,
                                        const std::vector<int>&);
template bool decode8bitImage<uint32_t>(Eigen::Ref<img_t<uint32_t>>,
                                        const ScanChannelData&,
                                        const std::vector<int>&);
template bool decode8bitImage<uint64_t>(Eigen::Ref<img_t<uint64_t>>,
                                        const ScanChannelData&,
                                        const std::vector<int>&);

template <typename T>
bool decode8bitImage(Eigen::Ref<img_t<T>> img,
                     const ScanChannelData& channel_buf) {
    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_read_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);
        return true;
    }

    VectorReader channel_reader(channel_buf);
    png_set_read_fn(png_ptr, &channel_reader, png_osf_read_data);

    // only used with Gray 16 bit to get little-endian LSB representation back
    // for other color types PNG_TRANSFORM_SWAP_ENDIAN does nothing
    int transforms = PNG_TRANSFORM_SWAP_ENDIAN;
    png_read_png(png_ptr, png_info_ptr, transforms, nullptr);

    png_uint_32 width;
    png_uint_32 height;
    int sample_depth;
    int color_type;

    png_get_IHDR(png_ptr, png_info_ptr, &width, &height, &sample_depth,
                 &color_type, nullptr, nullptr, nullptr);

    png_bytepp row_pointers = png_get_rows(png_ptr, png_info_ptr);

    // Sanity checks for encoded PNG size
    if (width != static_cast<png_uint_32>(img.cols()) ||
        height != static_cast<png_uint_32>(img.rows())) {
        std::cout << "ERROR: img contains data of incompatible size: " << width
                  << "x" << height << ", expected: " << img.cols() << "x"
                  << img.rows() << std::endl;
        return true;
    }

    if (sample_depth != 8) {
        std::cout << "ERROR: encoded img contains data with incompatible "
                     "sample_depth: "
                  << sample_depth << ", expected: 16" << std::endl;
        return true;
    }

    if (color_type != PNG_COLOR_TYPE_GRAY) {
        std::cout << "ERROR: encoded img contains data with incompatible "
                     "color type: "
                  << color_type << ", expected: " << PNG_COLOR_TYPE_GRAY
                  << std::endl;
        return true;
    }

    // 16bit channel data decoding to LidarScan for key channel_index
    for (size_t u = 0; u < height; u++) {
        for (size_t v = 0; v < width; v++) {
            img(u, v) = row_pointers[u][v];
        }
    }

    png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);

    return false;  // SUCCESS
}

template bool decode8bitImage<uint8_t>(Eigen::Ref<img_t<uint8_t>>,
                                       const ScanChannelData&);
template bool decode8bitImage<uint16_t>(Eigen::Ref<img_t<uint16_t>>,
                                        const ScanChannelData&);
template bool decode8bitImage<uint32_t>(Eigen::Ref<img_t<uint32_t>>,
                                        const ScanChannelData&);
template bool decode8bitImage<uint64_t>(Eigen::Ref<img_t<uint64_t>>,
                                        const ScanChannelData&);

// =================== Save to File Functions ====================

bool saveScanChannel(const ScanChannelData& channel_buf,
                     const std::string& filename) {
    std::fstream file(filename, std::ios_base::out | std::ios_base::binary);

    if (file.good()) {
        file.write(reinterpret_cast<const char*>(channel_buf.data()),
                   channel_buf.size());
        if (file.good()) {
            file.close();
            return false;  // SUCCESS
        }
    }

    file.close();
    return true;  // FAILURE
}

}  // namespace osf
}  // namespace ouster
