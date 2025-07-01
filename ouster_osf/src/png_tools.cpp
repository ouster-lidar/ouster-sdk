/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "png_tools.h"

#include <Eigen/Eigen>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>

#include "ouster/impl/logging.h"
#include "ouster/lidar_scan.h"

using namespace ouster::sensor;

namespace ouster {
namespace osf {

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
 * @TODO Change up tests to not use this stuff
 */
OUSTER_API_FUNCTION
void png_osf_error(png_structp png_ptr, png_const_charp msg) {
    logger().error("ERROR libpng osf: {}", msg);
    longjmp(png_jmpbuf(png_ptr), 1);
};

/** @internal */
inline void print_incompatable_image_size(png_uint_32 actual_width,
                                          png_uint_32 actual_height,
                                          png_uint_32 expected_width,
                                          png_uint_32 expected_height) {
    logger().error(
        "ERROR: img contains data of incompatible size: "
        " {}x{}, expected: {}x{}",
        actual_width, actual_height, expected_width, expected_height);
}

/** @internal */
inline void print_bad_sample_depth(int actual, int expected) {
    logger().error(
        "ERROR: encoded img contains data "
        "with incompatible sample_depth: {}"
        ", expected: {}",
        actual, expected);
}

/** @internal */
inline void print_bad_color_type(int actual, int expected) {
    logger().error(
        "ERROR: encoded img contains data with incompatible "
        "color type: {}, expected: {}",
        actual, expected);
}

inline void print_bad_pixel_size() {
    logger().error(
        "WARNING: Attempt to decode image"
        " of bigger pixel size");
}
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

/**
 * It's needed for custom png IO operations... but I've never seen it's called.
 * And also there are no need to flush writer to std::vector buffer in our case.
 * @TODO Change up tests to not use this stuff
 */
OUSTER_API_FUNCTION
void png_osf_flush_data(png_structp){};

/**
 * Common png WRITE init routine, creates and setups png_ptr and png_info_ptr
 */
bool png_osf_write_init(png_structpp png_ptrp, png_infopp png_info_ptrp) {
    *png_ptrp = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr,
                                        png_osf_error, png_osf_error);
    if (!*png_ptrp) {
        logger().error("ERROR: no png_ptr");
        return true;
    }

    *png_info_ptrp = png_create_info_struct(*png_ptrp);
    if (!*png_info_ptrp) {
        logger().error("ERROR: no png_info_ptr");
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
        logger().error("ERROR: no png_ptr");
        return true;
    }

    *png_info_ptrp = png_create_info_struct(*png_ptrp);
    if (!*png_info_ptrp) {
        logger().error("ERROR: no png_info_ptr");
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
                         ScanChannelData& res_buf, uint32_t width,
                         uint32_t height, int sample_depth, int color_type,
                         int compression_amount) {
    // Use setjmp() on upper level for errors catching
    png_set_write_fn(png_ptr, &res_buf, png_osf_write_data, png_osf_flush_data);

    png_set_compression_level(png_ptr, compression_amount);

    png_set_IHDR(png_ptr, png_info_ptr, width, height, sample_depth, color_type,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png_ptr, png_info_ptr);
}

// ========== Decode Functions ===================================

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
        print_incompatable_image_size(width, height,
                                      static_cast<png_uint_32>(img.cols()),
                                      static_cast<png_uint_32>(img.rows()));
        return true;
    }

    if (sample_depth != 8) {
        print_bad_sample_depth(sample_depth, 8);
        return true;
    }

    if (color_type != PNG_COLOR_TYPE_RGB) {
        print_bad_color_type(color_type, PNG_COLOR_TYPE_RGB);
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
        print_bad_pixel_size();
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
        print_incompatable_image_size(width, height,
                                      static_cast<png_uint_32>(img.cols()),
                                      static_cast<png_uint_32>(img.rows()));
        return true;
    }

    if (sample_depth != 8) {
        print_bad_sample_depth(sample_depth, 8);
        return true;
    }

    if (color_type != PNG_COLOR_TYPE_RGB_ALPHA) {
        print_bad_color_type(color_type, PNG_COLOR_TYPE_RGB_ALPHA);
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
        print_bad_pixel_size();
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
        print_incompatable_image_size(width, height,
                                      static_cast<png_uint_32>(img.cols()),
                                      static_cast<png_uint_32>(img.rows()));

        return true;
    }

    if (sample_depth != 16) {
        print_bad_sample_depth(sample_depth, 16);
        return true;
    }

    if (color_type != PNG_COLOR_TYPE_RGB_ALPHA) {
        print_bad_color_type(color_type, PNG_COLOR_TYPE_RGB_ALPHA);

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
        print_bad_pixel_size();
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
        print_incompatable_image_size(width, height,
                                      static_cast<png_uint_32>(img.cols()),
                                      static_cast<png_uint_32>(img.rows()));
        return true;
    }

    if (sample_depth != 16) {
        print_bad_sample_depth(sample_depth, 16);
        return true;
    }

    if (color_type != PNG_COLOR_TYPE_GRAY) {
        print_bad_color_type(color_type, PNG_COLOR_TYPE_GRAY);
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
        print_incompatable_image_size(width, height,
                                      static_cast<png_uint_32>(img.cols()),
                                      static_cast<png_uint_32>(img.rows()));
        return true;
    }

    if (sample_depth != 8) {
        print_bad_sample_depth(sample_depth, 8);
        return true;
    }

    if (color_type != PNG_COLOR_TYPE_GRAY) {
        print_bad_color_type(color_type, PNG_COLOR_TYPE_GRAY);
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

void decodeField(ouster::Field& field, const ScanChannelData& buffer,
                 const std::vector<int>& px_offset) {
    // 1d case, uncompressed
    if (field.shape().size() == 1) {
        std::memcpy(field, buffer.data(), buffer.size());
        return;
    }

    // empty case
    if (field.bytes() == 0) {
        return;
    }

    FieldView view = uint_view(field);
    // collapse shape
    if (view.shape().size() > 2) {
        size_t rows = view.shape()[0];
        size_t cols = view.size() / rows;
        view = view.reshape(rows, cols);
    }

    bool res = true;
    if (px_offset.size()) {
        switch (view.tag()) {
            case sensor::ChanFieldType::UINT8:
                res = decode8bitImage<uint8_t>(view, buffer, px_offset);
                break;
            case sensor::ChanFieldType::UINT16:
                res = decode16bitImage<uint16_t>(view, buffer, px_offset);
                break;
            case sensor::ChanFieldType::UINT32:
                res = decode32bitImage<uint32_t>(view, buffer, px_offset);
                break;
            case sensor::ChanFieldType::UINT64:
                res = decode64bitImage<uint64_t>(view, buffer, px_offset);
                break;
            default:
                break;
        }
    } else {
        switch (view.tag()) {
            case sensor::ChanFieldType::UINT8:
                res = decode8bitImage<uint8_t>(view, buffer);
                break;
            case sensor::ChanFieldType::UINT16:
                res = decode16bitImage<uint16_t>(view, buffer);
                break;
            case sensor::ChanFieldType::UINT32:
                res = decode32bitImage<uint32_t>(view, buffer);
                break;
            case sensor::ChanFieldType::UINT64:
                res = decode64bitImage<uint64_t>(view, buffer);
                break;
            default:
                break;
        }
    }

    if (res) {
        throw std::runtime_error("decodeField: could not decode field");
    }
}

}  // namespace osf
}  // namespace ouster
