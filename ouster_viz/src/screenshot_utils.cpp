/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved
 */
#include "screenshot_utils.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ouster/core/typedefs.h"
#include "ouster/io/image_io.h"

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {
namespace screenshot_utils {

namespace {

std::string generate_filename(const std::string& path) {
    // Get the current time
    auto now = std::chrono::system_clock::now();
    // Convert it to time_t for standard formatting
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // Get the milliseconds
    auto now_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream filename_ss;
    std::tm local_time;
#if defined(_WIND32) || defined(_MSC_VER)
    const char path_sep = '\\';
    localtime_s(&local_time, &now_c);  // Windows secure version
#else
    const char path_sep = '/';
    localtime_r(&now_c, &local_time);  // POSIX (Linux/macOS)
#endif
    auto final_path = path;
    if (!path.empty() && path.back() != path_sep) {
        final_path.push_back(path_sep);
    }

    filename_ss << final_path << "viz_screenshot_"
                << std::put_time(&local_time,
                                 "%Y-%m-%d_%H-%M-%S")                          // Date and time
                << "." << std::setw(3) << std::setfill('0') << now_ms.count()  // Milliseconds
                << ".png";
    return filename_ss.str();
}

}  // namespace

std::string write_png(const std::string& path, const std::vector<uint8_t>& pixels, int width,
                      int height) {
    if (width <= 0 || height <= 0 ||
        pixels.size() < static_cast<size_t>(width) * static_cast<size_t>(height) * 3) {
        // Invalid image dimensions or missing pixel data
        return "";
    }

    auto filename = generate_filename(path);
    // Viz colors are authored as sRGB, and on macOS the window is tagged as
    // sRGB (see glfw_macos.mm), so the PNG must keep the sRGB tag.
    // Recording writes a PNG every frame, so skip row filtering for speed.
    io::PngWriteOptions options;
    options.srgb = true;
    options.filter_rows = false;
    const Eigen::TensorMap<const core::rgb_img_t<uint8_t>> image(pixels.data(), height, width, 3);
    io::write_png(filename, image, options);
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
        uint8_t* top_row = pixels.data() + (row * row_size);
        uint8_t* bottom_row = pixels.data() + ((height - row - 1) * row_size);

        // Swap the two rows
        std::memcpy(tmp.data(), top_row, row_size);
        std::memcpy(top_row, bottom_row, row_size);
        std::memcpy(bottom_row, tmp.data(), row_size);
    }
}

}  // namespace screenshot_utils
}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster
