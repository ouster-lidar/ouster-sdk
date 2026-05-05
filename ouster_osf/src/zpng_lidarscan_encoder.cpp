/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/zpng_lidarscan_encoder.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "ouster/impl/logging.h"
#include "zpng.h"

using namespace ouster::sdk::core;

namespace ouster {
namespace sdk {
namespace osf {

ScanChannelData ZPngLidarScanEncoder::encode_field(
    const ouster::sdk::core::Field& field,
    const std::vector<int>& /*px_offset*/) const {
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

    opts.Buffer.Data = static_cast<unsigned char*>(view.get());
    opts.Buffer.Bytes = view.bytes();

    int channels = 1;
    int bytewidth = 1;
    switch (view.tag()) {
        case ChanFieldType::UINT8:
            break;
        case ChanFieldType::UINT16:
            bytewidth = 2;
            break;
        case ChanFieldType::UINT32:
            channels = 4;
            break;
        case ChanFieldType::UINT64:
            channels = 4;
            bytewidth = 2;
            break;
        default:
            throw std::runtime_error("Unhandled chanfield type.");
    }

    opts.BytesPerChannel = bytewidth;
    opts.Channels = channels;
    opts.StrideBytes = opts.WidthPixels * channels * bytewidth;

    ZPNG_Allocator alloc;
    alloc.AllocatorData = &buffer;
    alloc.Allocator = [](uint64_t size, void* data) {
        auto vec = static_cast<std::vector<uint8_t>*>(data);
        vec->resize(size);
        return static_cast<void*>(vec->data());
    };

    auto out = ZPNG_CompressEx(&opts, &alloc, compression_amount_);
    if (out.Data == nullptr) {
        throw std::runtime_error("Failed to compress zpng.");
    }
    buffer.resize(out.Bytes);  // resize to actual size

    return buffer;
}
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
