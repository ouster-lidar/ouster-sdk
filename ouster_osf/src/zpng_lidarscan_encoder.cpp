/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/zpng_lidarscan_encoder.h"

#include <png.h>

#include "ouster/impl/logging.h"
#include "png_tools.h"
#include "zpng.h"

using namespace ouster::sensor;

namespace ouster {
namespace osf {

bool ZPngLidarScanEncoder::fieldEncode(const LidarScan& lidar_scan,
                                       const ouster::FieldType& field_type,
                                       const std::vector<int>& /*px_offset*/,
                                       ScanData& scan_data,
                                       size_t scan_idx) const {
    if (scan_idx >= scan_data.size()) {
        throw std::invalid_argument(
            "ERROR: scan_data size is not sufficient to hold idx: " +
            std::to_string(scan_idx));
    }

    ZPNG_ImageData opts;
    opts.HeightPixels = lidar_scan.h;
    opts.WidthPixels = lidar_scan.w;

    auto& field = lidar_scan.field(field_type.name);

    opts.Buffer.Data = (unsigned char*)field.get();
    opts.Buffer.Bytes = field.bytes();

    int channels = 1;
    int bytewidth = 1;
    switch (field_type.element_type) {
        case sensor::ChanFieldType::UINT8:
            break;
        case sensor::ChanFieldType::UINT16:
            bytewidth = 2;
            break;
        case sensor::ChanFieldType::UINT32:
            channels = 4;
            break;
        case sensor::ChanFieldType::UINT64:
            channels = 4;
            bytewidth = 2;
            break;
        default:
            throw std::runtime_error("Unhandled chanfield type.");
    }

    opts.BytesPerChannel = bytewidth;
    opts.Channels = channels;
    int stride = lidar_scan.w * channels * bytewidth;
    opts.StrideBytes = stride;

    ZPNG_Allocator alloc;
    alloc.AllocatorData = &scan_data[scan_idx];
    alloc.Allocator = [](uint64_t size, void* data) {
        auto vec = static_cast<std::vector<uint8_t>*>(data);
        vec->resize(size);
        return static_cast<void*>(vec->data());
    };

    auto out = ZPNG_CompressEx(&opts, &alloc);
    if (!out.Data) {
        throw std::runtime_error("Failed to compress zpng.");
    }
    scan_data[scan_idx].resize(out.Bytes);  // resize to actual size
    return false;
}

ScanChannelData ZPngLidarScanEncoder::encodeField(
    const ouster::Field& field) const {
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

    ZPNG_ImageData opts;
    opts.HeightPixels = view.shape()[0];
    opts.WidthPixels = view.shape()[1];

    opts.Buffer.Data = (unsigned char*)view.get();
    opts.Buffer.Bytes = view.bytes();

    int channels = 1;
    int bytewidth = 1;
    switch (view.tag()) {
        case sensor::ChanFieldType::UINT8:
            break;
        case sensor::ChanFieldType::UINT16:
            bytewidth = 2;
            break;
        case sensor::ChanFieldType::UINT32:
            channels = 4;
            break;
        case sensor::ChanFieldType::UINT64:
            channels = 4;
            bytewidth = 2;
            break;
        default:
            throw std::runtime_error("Unhandled chanfield type.");
    }

    opts.BytesPerChannel = bytewidth;
    opts.Channels = channels;
    int stride = opts.WidthPixels * channels * bytewidth;
    opts.StrideBytes = stride;

    ZPNG_Allocator alloc;
    alloc.AllocatorData = &buffer;
    alloc.Allocator = [](uint64_t size, void* data) {
        auto vec = static_cast<std::vector<uint8_t>*>(data);
        vec->resize(size);
        return static_cast<void*>(vec->data());
    };

    auto out = ZPNG_CompressEx(&opts, &alloc);
    if (!out.Data) {
        throw std::runtime_error("Failed to compress zpng.");
    }
    buffer.resize(out.Bytes);  // resize to actual size

    return buffer;
}
}  // namespace osf
}  // namespace ouster
