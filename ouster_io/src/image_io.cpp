/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 */

#include "ouster/io/image_io.h"

#include <png.h>
#include <pngconf.h>

#include <algorithm>
#include <cmath>
#include <csetjmp>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <ios>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ouster/core/typedefs.h"

#define MSF_GIF_IMPL
#include "msf_gif/msf_gif.h"

namespace ouster {
namespace sdk {
namespace io {

namespace {

// libpng reports errors by longjmp-ing back to the setjmp of the active call.
// Functions that call setjmp must therefore only hold trivially destructible
// locals, and everything they use is passed in through this struct.
struct PngContext {
    char message[256];
    const uint8_t* data;
    size_t size;
    size_t offset;
    std::ofstream* out;
};

void png_error_fn(png_structp png, png_const_charp msg) {
    auto* ctx = static_cast<PngContext*>(png_get_error_ptr(png));
    (void)std::snprintf(ctx->message, sizeof(ctx->message), "%s",
                        msg != nullptr ? msg : "unknown libpng error");
    // NOLINTNEXTLINE(cert-err52-cpp)
    longjmp(png_jmpbuf(png), 1);
}

void png_warning_fn(png_structp /*png*/, png_const_charp /*msg*/) {}

void png_read_fn(png_structp png, png_bytep out, png_size_t length) {
    auto* ctx = static_cast<PngContext*>(png_get_io_ptr(png));
    if (length > ctx->size - ctx->offset) {
        png_error(png, "unexpected end of file");
    }
    std::memcpy(out, ctx->data + ctx->offset, length);
    ctx->offset += length;
}

void png_write_fn(png_structp png, png_bytep data, png_size_t length) {
    auto* ctx = static_cast<PngContext*>(png_get_io_ptr(png));
    ctx->out->write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(length));
    if (!*ctx->out) {
        png_error(png, "failed to write to file");
    }
}

void png_flush_fn(png_structp png) {
    static_cast<PngContext*>(png_get_io_ptr(png))->out->flush();
}

bool png_read_header(png_structp png, png_infop info, size_t* width, size_t* height,
                     size_t* channels) {
    // NOLINTNEXTLINE(cert-err52-cpp)
    if (setjmp(png_jmpbuf(png))) {
        return false;
    }
    png_read_info(png, info);
    png_set_expand(png);
    if (png_get_bit_depth(png, info) == 16) {
#ifdef PNG_READ_SCALE_16_TO_8_SUPPORTED
        png_set_scale_16(png);
#else
        png_set_strip_16(png);
#endif
    }
    png_set_interlace_handling(png);
    png_read_update_info(png, info);
    *width = png_get_image_width(png, info);
    *height = png_get_image_height(png, info);
    *channels = png_get_channels(png, info);
    return true;
}

bool png_read_rows(png_structp png, png_infop info, png_bytepp rows) {
    // NOLINTNEXTLINE(cert-err52-cpp)
    if (setjmp(png_jmpbuf(png))) {
        return false;
    }
    png_read_image(png, rows);
    png_read_end(png, info);
    return true;
}

bool png_write_rows(png_structp png, png_infop info, png_bytepp rows, png_uint_32 width,
                    png_uint_32 height, int color_type, bool srgb, bool filter_rows) {
    // NOLINTNEXTLINE(cert-err52-cpp)
    if (setjmp(png_jmpbuf(png))) {
        return false;
    }
    png_set_IHDR(png, info, width, height, 8, color_type, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    if (srgb) {
        png_set_sRGB_gAMA_and_cHRM(png, info, PNG_sRGB_INTENT_PERCEPTUAL);
    }
    png_write_info(png, info);
    if (!filter_rows) {
        png_set_filter(png, PNG_FILTER_TYPE_BASE, PNG_FILTER_NONE);
    }
    png_write_image(png, rows);
    png_write_end(png, nullptr);
    return true;
}

struct PngReadGuard {
    png_structp png{nullptr};
    png_infop info{nullptr};
    PngReadGuard() = default;
    PngReadGuard(const PngReadGuard&) = delete;
    PngReadGuard& operator=(const PngReadGuard&) = delete;
    PngReadGuard(PngReadGuard&&) = delete;
    PngReadGuard& operator=(PngReadGuard&&) = delete;
    ~PngReadGuard() {
        if (png != nullptr) {
            png_destroy_read_struct(&png, info != nullptr ? &info : nullptr, nullptr);
        }
    }
};

struct PngWriteGuard {
    png_structp png{nullptr};
    png_infop info{nullptr};
    PngWriteGuard() = default;
    PngWriteGuard(const PngWriteGuard&) = delete;
    PngWriteGuard& operator=(const PngWriteGuard&) = delete;
    PngWriteGuard(PngWriteGuard&&) = delete;
    PngWriteGuard& operator=(PngWriteGuard&&) = delete;
    ~PngWriteGuard() {
        if (png != nullptr) {
            png_destroy_write_struct(&png, info != nullptr ? &info : nullptr);
        }
    }
};

int png_color_type(size_t channels) {
    switch (channels) {
        case 1:
            return PNG_COLOR_TYPE_GRAY;
        case 2:
            return PNG_COLOR_TYPE_GRAY_ALPHA;
        case 3:
            return PNG_COLOR_TYPE_RGB;
        case 4:
            return PNG_COLOR_TYPE_RGB_ALPHA;
        default:
            throw std::invalid_argument("write_png: channels must be 1, 2, 3 or 4, got " +
                                        std::to_string(channels));
    }
}

size_t gif_file_write(const void* buffer, size_t size, size_t count, void* stream) {
    auto* file = static_cast<std::ofstream*>(stream);
    file->write(static_cast<const char*>(buffer), static_cast<std::streamsize>(size * count));
    return *file ? count : 0;
}

// limits enforced by the GIF format and by msf_gif_begin
constexpr size_t GIF_MAX_DIMENSION = 65535;
constexpr size_t GIF_MAX_PIXELS = 268435456;
constexpr int GIF_QUALITY = 16;

}  // namespace

core::rgb_img_t<uint8_t> read_png(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        throw std::runtime_error("read_png: failed to open " + filename);
    }
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)),
                               std::istreambuf_iterator<char>());
    constexpr size_t signature_size = 8;
    if (bytes.size() < signature_size || png_sig_cmp(bytes.data(), 0, signature_size) != 0) {
        throw std::runtime_error("read_png: " + filename + " is not a PNG file");
    }

    PngContext ctx{};
    ctx.data = bytes.data();
    ctx.size = bytes.size();

    PngReadGuard guard;
    guard.png = png_create_read_struct(PNG_LIBPNG_VER_STRING, &ctx, png_error_fn, png_warning_fn);
    if (guard.png == nullptr) {
        throw std::runtime_error("read_png: failed to create png read struct");
    }
    guard.info = png_create_info_struct(guard.png);
    if (guard.info == nullptr) {
        throw std::runtime_error("read_png: failed to create png info struct");
    }
    png_set_read_fn(guard.png, &ctx, png_read_fn);

    size_t width = 0;
    size_t height = 0;
    size_t channels = 0;
    core::rgb_img_t<uint8_t> image;
    bool ok = png_read_header(guard.png, guard.info, &width, &height, &channels);
    if (ok) {
        image.resize(static_cast<Eigen::Index>(height), static_cast<Eigen::Index>(width),
                     static_cast<Eigen::Index>(channels));
        const size_t row_size = width * channels;
        std::vector<png_bytep> rows(height);
        for (size_t y = 0; y < height; ++y) {
            rows[y] = image.data() + (y * row_size);
        }
        ok = png_read_rows(guard.png, guard.info, rows.data());
    }
    if (!ok) {
        throw std::runtime_error("read_png: failed to read " + filename + ": " + ctx.message);
    }
    return image;
}

void write_png(const std::string& filename, Eigen::TensorMap<const core::rgb_img_t<uint8_t>> image,
               const PngWriteOptions& options) {
    const Eigen::Index height_index = image.dimension(0);
    const Eigen::Index width_index = image.dimension(1);
    if (image.data() == nullptr || width_index <= 0 || height_index <= 0 ||
        width_index > PNG_UINT_31_MAX || height_index > PNG_UINT_31_MAX) {
        throw std::invalid_argument("write_png: invalid image dimensions " +
                                    std::to_string(width_index) + "x" +
                                    std::to_string(height_index));
    }
    const auto width = static_cast<size_t>(width_index);
    const auto height = static_cast<size_t>(height_index);
    const auto channels = static_cast<size_t>(std::max<Eigen::Index>(0, image.dimension(2)));
    const uint8_t* pixels = image.data();
    const int color_type = png_color_type(channels);

    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        throw std::runtime_error("write_png: failed to open " + filename + " for writing");
    }

    PngContext ctx{};
    ctx.out = &file;

    PngWriteGuard guard;
    guard.png = png_create_write_struct(PNG_LIBPNG_VER_STRING, &ctx, png_error_fn, png_warning_fn);
    if (guard.png == nullptr) {
        throw std::runtime_error("write_png: failed to create png write struct");
    }
    guard.info = png_create_info_struct(guard.png);
    if (guard.info == nullptr) {
        throw std::runtime_error("write_png: failed to create png info struct");
    }
    png_set_write_fn(guard.png, &ctx, png_write_fn, png_flush_fn);

    const size_t row_size = width * channels;
    std::vector<png_bytep> rows(height);
    for (size_t y = 0; y < height; ++y) {
        // libpng doesn't modify the rows it writes but takes them as non-const
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        rows[y] = const_cast<png_bytep>(pixels + (y * row_size));
    }
    if (!png_write_rows(guard.png, guard.info, rows.data(), static_cast<png_uint_32>(width),
                        static_cast<png_uint_32>(height), color_type, options.srgb,
                        options.filter_rows)) {
        throw std::runtime_error("write_png: failed to write " + filename + ": " + ctx.message);
    }
    file.close();
    if (!file) {
        throw std::runtime_error("write_png: failed to write " + filename);
    }
}

struct GifWriter::Impl {
    std::ofstream file;
    MsfGifState state{};
    size_t width{0};
    size_t height{0};
    std::vector<uint8_t> rgba;
    double elapsed_ms{0.0};
    int64_t elapsed_cs{0};
    bool open{false};
};

GifWriter::GifWriter(const std::string& filename, size_t width, size_t height)
    : impl_(std::make_unique<Impl>()) {
    if (width == 0 || height == 0 || width > GIF_MAX_DIMENSION || height > GIF_MAX_DIMENSION ||
        width * height >= GIF_MAX_PIXELS) {
        throw std::invalid_argument("GifWriter: invalid image dimensions " + std::to_string(width) +
                                    "x" + std::to_string(height));
    }
    impl_->file.open(filename, std::ios::binary);
    if (!impl_->file) {
        throw std::runtime_error("GifWriter: failed to open " + filename + " for writing");
    }
    impl_->width = width;
    impl_->height = height;
    impl_->rgba.resize(width * height * 4);
    if (msf_gif_begin_to_file(&impl_->state, static_cast<int>(width), static_cast<int>(height),
                              gif_file_write, &impl_->file) == 0) {
        throw std::runtime_error("GifWriter: failed to start encoding " + filename);
    }
    impl_->open = true;
}

GifWriter::~GifWriter() {
    if (impl_ && impl_->open) {
        msf_gif_end_to_file(&impl_->state);
    }
}

void GifWriter::add_frame(Eigen::TensorMap<const core::rgb_img_t<uint8_t>> frame,
                          double duration_ms) {
    if (!impl_->open) {
        throw std::runtime_error("GifWriter: add_frame called after close");
    }
    const Eigen::Index channels = frame.dimension(2);
    if (frame.data() == nullptr || frame.dimension(0) != static_cast<Eigen::Index>(impl_->height) ||
        frame.dimension(1) != static_cast<Eigen::Index>(impl_->width) ||
        (channels != 3 && channels != 4)) {
        const std::string height = std::to_string(impl_->height);
        const std::string width = std::to_string(impl_->width);
        throw std::invalid_argument("GifWriter.add_frame: frame must have shape (" + height + ", " +
                                    width + ", 3) or (" + height + ", " + width + ", 4)");
    }
    const uint8_t* pixels = frame.data();

    const size_t count = impl_->width * impl_->height;
    uint8_t* out = impl_->rgba.data();
    if (channels == 4) {
        std::memcpy(out, pixels, count * 4);
    } else {
        for (size_t i = 0; i < count; ++i) {
            out[(i * 4) + 0] = pixels[(i * 3) + 0];
            out[(i * 4) + 1] = pixels[(i * 3) + 1];
            out[(i * 4) + 2] = pixels[(i * 3) + 2];
            out[(i * 4) + 3] = 255;
        }
    }

    impl_->elapsed_ms += duration_ms > 0.0 ? duration_ms : 0.0;
    const auto target_cs = static_cast<int64_t>(std::llround(impl_->elapsed_ms / 10.0));
    const int64_t delay_cs = std::max<int64_t>(1, target_cs - impl_->elapsed_cs);
    impl_->elapsed_cs += delay_cs;

    if (msf_gif_frame_to_file(&impl_->state, out, static_cast<int>(delay_cs), GIF_QUALITY,
                              static_cast<int>(impl_->width * 4)) == 0) {
        impl_->open = false;
        throw std::runtime_error("GifWriter: failed to encode frame");
    }
}

void GifWriter::close() {
    if (!impl_->open) {
        return;
    }
    impl_->open = false;
    const bool ok = msf_gif_end_to_file(&impl_->state) != 0;
    impl_->file.close();
    if (!ok || !impl_->file) {
        throw std::runtime_error("GifWriter: failed to write file");
    }
}

size_t GifWriter::width() const {
    return impl_->width;
}

size_t GifWriter::height() const {
    return impl_->height;
}

}  // namespace io
}  // namespace sdk
}  // namespace ouster
