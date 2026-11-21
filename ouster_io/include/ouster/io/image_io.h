/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace io {

/**
 * Read a PNG file into an 8 bit per channel image.
 *
 * Palette images are expanded to RGB, transparency chunks to an alpha
 * channel, gray images with fewer than 8 bits to 8 bits, and 16 bit images
 * are scaled down to 8 bits. Gray images stay gray.
 *
 * @param[in] filename the PNG file to read.
 * @return the decoded image, with shape (height, width, channels) where
 *         channels is 1 (gray), 2 (gray, alpha), 3 (RGB) or 4 (RGBA).
 * @throws std::runtime_error if the file can't be read or isn't a valid PNG.
 */
OUSTER_API_FUNCTION
core::rgb_img_t<uint8_t> read_png(const std::string& filename);

/** Options for writing PNG files. */
struct OUSTER_API_CLASS PngWriteOptions {
    /**
     * Tag the image as sRGB, so viewers on every platform and display
     * profile show the same colors.
     */
    bool srgb{true};

    /**
     * Filter rows before compressing them, which gives smaller files. Turn
     * off when writing speed matters more than file size.
     */
    bool filter_rows{true};
};

/**
 * Write an 8 bit per channel image to a PNG file.
 *
 * @param[in] filename the PNG file to write; overwritten if it exists.
 * @param[in] image the image, with shape (height, width, channels) where
 *            channels is 1 (gray), 2 (gray, alpha), 3 (RGB) or 4 (RGBA).
 * @param[in] options how to write the file.
 * @throws std::invalid_argument if the dimensions or channel count are invalid.
 * @throws std::runtime_error if the file can't be written.
 */
OUSTER_API_FUNCTION
void write_png(const std::string& filename, Eigen::TensorMap<const core::rgb_img_t<uint8_t>> image,
               const PngWriteOptions& options = PngWriteOptions{});

/**
 * Writes an animated GIF one frame at a time, so only a single frame is held
 * in memory while encoding. Each frame gets its own palette; frames loop
 * forever.
 */
class OUSTER_API_CLASS GifWriter {
   public:
    /**
     * Open a GIF file for writing.
     *
     * @param[in] filename the GIF file to write; overwritten if it exists.
     * @param[in] width width in pixels that every frame must have.
     * @param[in] height height in pixels that every frame must have.
     * @throws std::invalid_argument if the dimensions are invalid.
     * @throws std::runtime_error if the file can't be opened.
     */
    OUSTER_API_FUNCTION
    GifWriter(const std::string& filename, size_t width, size_t height);

    /** Finishes the file if close() wasn't called; errors are ignored. */
    OUSTER_API_FUNCTION
    ~GifWriter();

    GifWriter(const GifWriter&) = delete;
    GifWriter& operator=(const GifWriter&) = delete;
    GifWriter(GifWriter&&) = delete;
    GifWriter& operator=(GifWriter&&) = delete;

    /**
     * Encode and append a frame.
     *
     * GIF delays are stored in hundredths of a second. Rounding is carried
     * over between frames so the total duration doesn't drift, and every frame
     * is shown for at least one hundredth of a second.
     *
     * @param[in] frame the frame, with shape (height, width, 3) for RGB or
     *            (height, width, 4) for RGBA; alpha is ignored.
     * @param[in] duration_ms how long to show the frame, in milliseconds.
     * @throws std::invalid_argument if the frame shape doesn't match the writer.
     * @throws std::runtime_error if the writer is closed or encoding fails.
     */
    OUSTER_API_FUNCTION
    void add_frame(Eigen::TensorMap<const core::rgb_img_t<uint8_t>> frame, double duration_ms);

    /**
     * Write the end of the file and close it. Further calls have no effect.
     *
     * @throws std::runtime_error if the file can't be written.
     */
    OUSTER_API_FUNCTION
    void close();

    /** @return the frame width in pixels. */
    OUSTER_API_FUNCTION
    size_t width() const;

    /** @return the frame height in pixels. */
    OUSTER_API_FUNCTION
    size_t height() const;

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace io
}  // namespace sdk
}  // namespace ouster
