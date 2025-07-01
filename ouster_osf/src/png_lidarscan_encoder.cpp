/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

// NOTE yes, we're aware that there is a high amount of duplication in this
// file. Some of the methods are "legacy" code and unlikely to change in future
// revisions.

#include "ouster/osf/png_lidarscan_encoder.h"

#include <png.h>

#include "ouster/impl/logging.h"
#include "png_tools.h"

/* Check for the older version of libpng and add missing macros as necessary */

#if (PNG_LIBPNG_VER_MAJOR == 1)
#if (PNG_LIBPNG_VER_MINOR < 5)
#define LIBPNG_VERSION_12
typedef png_bytep png_const_bytep;
#endif
#if (PNG_LIBPNG_VER_MINOR < 6)
typedef png_structp png_const_structrp;
typedef png_infop png_const_inforp;
typedef png_bytep png_const_bytep;
#endif
#endif

using namespace ouster::sensor;

namespace ouster {
namespace osf {

ScanChannelData PngLidarScanEncoder::encodeField(
    const ouster::Field& field, const std::vector<int>& px_offset) const {
    ScanChannelData buffer;

    // do not compress, flat fields "compressed" size is greater than original
    if (field.shape().size() == 1) {
        buffer.resize(field.bytes());
        std::memcpy(buffer.data(), field, field.bytes());
        return buffer;
    }

    // empty case
    if (field.bytes() == 0) {
        return buffer;
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
                res = encode8bitImage<uint8_t>(buffer, view, px_offset);
                break;
            case sensor::ChanFieldType::UINT16:
                res = encode16bitImage<uint16_t>(buffer, view, px_offset);
                break;
            case sensor::ChanFieldType::UINT32:
                res = encode32bitImage<uint32_t>(buffer, view, px_offset);
                break;
            case sensor::ChanFieldType::UINT64:
                res = encode64bitImage<uint64_t>(buffer, view, px_offset);
                break;
            default:
                break;
        }
    } else {
        switch (view.tag()) {
            case sensor::ChanFieldType::UINT8:
                res = encode8bitImage<uint8_t>(buffer, view);
                break;
            case sensor::ChanFieldType::UINT16:
                res = encode16bitImage<uint16_t>(buffer, view);
                break;
            case sensor::ChanFieldType::UINT32:
                res = encode32bitImage<uint32_t>(buffer, view);
                break;
            case sensor::ChanFieldType::UINT64:
                res = encode64bitImage<uint64_t>(buffer, view);
                break;
            default:
                break;
        }
    }

    if (res) {
        throw std::runtime_error("encodeField: could not encode field");
    }

    return buffer;
}

template <typename T>
bool PngLidarScanEncoder::encode8bitImage(
    ScanChannelData& res_buf, const Eigen::Ref<const img_t<T>>& img,
    const std::vector<int>& px_offset) const {
    return PngLidarScanEncoder::encode8bitImage<T>(res_buf,
                                                   destagger(img, px_offset));
}

template <typename T>
bool PngLidarScanEncoder::encode8bitImage(
    ScanChannelData& res_buf, const Eigen::Ref<const img_t<T>>& img) const {
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
                        sample_depth, color_type, compression_amount_);

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

template <typename T>
bool PngLidarScanEncoder::encode16bitImage(
    ScanChannelData& res_buf, const Eigen::Ref<const img_t<T>>& img) const {
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
                        sample_depth, color_type, compression_amount_);

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

template <typename T>
bool PngLidarScanEncoder::encode16bitImage(
    ScanChannelData& res_buf, const Eigen::Ref<const img_t<T>>& img,
    const std::vector<int>& px_offset) const {
    return encode16bitImage<T>(res_buf, destagger(img, px_offset));
}

template <typename T>
bool PngLidarScanEncoder::encode24bitImage(
    ScanChannelData& res_buf, const Eigen::Ref<const img_t<T>>& img,
    const std::vector<int>& px_offset) const {
    return encode24bitImage<T>(res_buf, destagger(img, px_offset));
}

template <typename T>
bool PngLidarScanEncoder::encode24bitImage(
    ScanChannelData& res_buf, const Eigen::Ref<const img_t<T>>& img) const {
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
                        sample_depth, color_type, compression_amount_);

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

template <typename T>
bool PngLidarScanEncoder::encode32bitImage(
    ScanChannelData& res_buf, const Eigen::Ref<const img_t<T>>& img,
    const std::vector<int>& px_offset) const {
    return encode32bitImage<T>(res_buf, destagger(img, px_offset));
}

template <typename T>
bool PngLidarScanEncoder::encode32bitImage(
    ScanChannelData& res_buf, const Eigen::Ref<const img_t<T>>& img) const {
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
                        sample_depth, color_type, compression_amount_);

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

template <typename T>
bool PngLidarScanEncoder::encode64bitImage(
    ScanChannelData& res_buf, const Eigen::Ref<const img_t<T>>& img,
    const std::vector<int>& px_offset) const {
    return encode64bitImage<T>(res_buf, destagger(img, px_offset));
}

template <typename T>
bool PngLidarScanEncoder::encode64bitImage(
    ScanChannelData& res_buf, const Eigen::Ref<const img_t<T>>& img) const {
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
                        sample_depth, color_type, compression_amount_);

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

}  // namespace osf
}  // namespace ouster
