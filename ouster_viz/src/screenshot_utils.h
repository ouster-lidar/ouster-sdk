/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved
 */
#pragma once

#include <string>
#include <vector>

namespace ouster {
namespace viz {
namespace impl {
namespace screenshot_utils {
/**
 * @brief Writes a PNG to the file system.
 * @param[in] path A string with the path, use empty string for current path
 * @param[in] pixels A vector of bytes with RGB data
 * @param[in] width The image width
 * @param[in] height The image height
 * @return The file name that was generated, or an empty string if there was
 * an error.
 */
std::string write_png(const std::string& path,
                      const std::vector<uint8_t>& pixels, int width,
                      int height);

/** @brief OpenGL framebuffers store pixels starting from the bottom-left
 * corner. Libpng expects pixels starting from the top left corner. This
 * function flips the pixels in place.
 * @param[in] pixels A vector of bytes with the RGB data
 * @param[in] width The image width
 * @param[in] height The image height
 */
void flip_pixels(std::vector<uint8_t>& pixels, int width, int height);

}  // namespace screenshot_utils

}  // namespace impl
}  // namespace viz
}  // namespace ouster
