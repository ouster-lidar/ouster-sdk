/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved
 */
#include "screenshot_utils.h"

#include <png.h>

#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace ouster {
namespace viz {
namespace impl {
namespace screenshot_utils {

namespace {

std::string generate_filename(const std::string& path) {
    // Get the current time
    auto now = std::chrono::system_clock::now();
    // Convert itto time_t for standard formatting
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // Get the milliseconds
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) %
                  1000;

    std::ostringstream filename_ss;
    std::tm local_time;
#if defined(_WIND32) || defined(_MSC_VER)
    const char PATH_SEP = '\\';
    localtime_s(&local_time, &now_c);  // Windows secure version
#else
    const char PATH_SEP = '/';
    localtime_r(&now_c, &local_time);  // POSIX (Linux/macOS)
#endif
    auto final_path = path;
    if (!path.empty() && path.back() != PATH_SEP) {
        final_path.push_back(PATH_SEP);
    }

    filename_ss << final_path << "viz_screenshot_"
                << std::put_time(&local_time,
                                 "%Y-%m-%d_%H-%M-%S")  // Date and time
                << "." << std::setw(3) << std::setfill('0')
                << now_ms.count()  // Milliseconds
                << ".png";
    return filename_ss.str();
}

}  // namespace

std::string write_png(const std::string& path,
                      const std::vector<uint8_t>& pixels, int width,
                      int height) {
    if (pixels.empty() || width <= 0 || height <= 0) {
        // Invalid image dimensions or empty pixel data
        return "";
    }

    auto filename = generate_filename(path);
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        throw std::runtime_error("write_png: Failed to open file for writing");
    }

    // Set up libpng
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr,
                                              nullptr, nullptr);
    if (!png) {
        throw std::runtime_error(
            "write_png: Failed to create png write struct");
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_write_struct(&png, nullptr);
        throw std::runtime_error("write_png: Failed to create png info struct");
    }

    if (setjmp(png_jmpbuf(png))) {
        png_destroy_write_struct(&png, &info);
        throw std::runtime_error("write_png: Error during PNG creation");
    }

    auto write_fn = [](png_structp png_ptr, png_bytep data, png_size_t length) {
        auto* file = static_cast<std::ofstream*>(png_get_io_ptr(png_ptr));
        file->write(reinterpret_cast<const char*>(data), length);
    };

    auto flush_fn = [](png_structp png_ptr) {
        auto* file = static_cast<std::ofstream*>(png_get_io_ptr(png_ptr));
        file->flush();
    };

    png_set_write_fn(png, &file, write_fn, flush_fn);

    png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png, info);
    png_set_filter(png, 0, PNG_FILTER_NONE);

    std::vector<png_bytep> row_pointers(height);
    for (int y = 0; y < height; ++y) {
        row_pointers[y] = (png_bytep)&pixels[y * width * 3];
    }

    png_write_image(png, row_pointers.data());
    png_write_end(png, nullptr);

    png_destroy_write_struct(&png, &info);
    file.close();
    return filename;
}

void flip_pixels(std::vector<uint8_t>& pixels, int width, int height) {
    // Each row is width * 3 bytes (for RGB)
    std::size_t row_size = static_cast<std::size_t>(width) * 3;

    // Ensure the vector is large enough
    if (pixels.size() < row_size * height) {
        throw std::runtime_error(
            "flip_pixels: Data size is less "
            "than the required size for the specified "
            "width and height");
    }

    // Temporary buffer to swap rows
    std::vector<uint8_t> tmp(row_size);

    // Perform the vertical flip in place
    for (int row = 0; row < height / 2; ++row) {
        // Indices for the current row from the top and its counterpart from the
        // bottom
        uint8_t* top_row = pixels.data() + row * row_size;
        uint8_t* bottom_row = pixels.data() + (height - row - 1) * row_size;

        // Swap the two rows
        std::memcpy(tmp.data(), top_row, row_size);
        std::memcpy(top_row, bottom_row, row_size);
        std::memcpy(bottom_row, tmp.data(), row_size);
    }
}

}  // namespace screenshot_utils
}  // namespace impl
}  // namespace viz
}  // namespace ouster
