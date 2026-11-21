/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved
 */
#include "ouster/io/image_io.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <ios>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "ouster/core/typedefs.h"

using ouster::sdk::core::rgb_img_t;
using ouster::sdk::io::GifWriter;
using ouster::sdk::io::PngWriteOptions;
using ouster::sdk::io::read_png;
using ouster::sdk::io::write_png;
using ImageMap = Eigen::TensorMap<const rgb_img_t<uint8_t>>;

namespace {

std::string temp_path(const std::string& name) {
    return ::testing::TempDir() + name;
}

std::vector<uint8_t> read_file(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

// Chunk types of a PNG file, in file order.
std::vector<std::string> png_chunk_types(const std::vector<uint8_t>& data) {
    std::vector<std::string> types;
    size_t pos = 8;
    while (pos + 8 <= data.size()) {
        const size_t length = (static_cast<size_t>(data[pos]) << 24) |
                              (static_cast<size_t>(data[pos + 1]) << 16) |
                              (static_cast<size_t>(data[pos + 2]) << 8) | data[pos + 3];
        types.emplace_back(data.begin() + static_cast<std::ptrdiff_t>(pos) + 4,
                           data.begin() + static_cast<std::ptrdiff_t>(pos) + 8);
        pos += 12 + length;
    }
    return types;
}

constexpr Eigen::Index WIDTH = 4;
constexpr Eigen::Index HEIGHT = 3;

rgb_img_t<uint8_t> test_image(Eigen::Index channels, Eigen::Index width = WIDTH) {
    rgb_img_t<uint8_t> image(HEIGHT, width, channels);
    for (Eigen::Index i = 0; i < image.size(); ++i) {
        image.data()[i] = static_cast<uint8_t>((i * 37) % 256);
    }
    return image;
}

bool images_equal(const rgb_img_t<uint8_t>& a, const rgb_img_t<uint8_t>& b) {
    return a.dimensions() == b.dimensions() && std::equal(a.data(), a.data() + a.size(), b.data());
}

void expect_png_round_trip(Eigen::Index channels, bool filter_rows) {
    const auto filename = temp_path("image_io_round_trip.png");
    const auto image = test_image(channels);
    PngWriteOptions options;
    options.filter_rows = filter_rows;
    write_png(filename, image, options);

    const auto result = read_png(filename);
    EXPECT_EQ(result.dimension(0), HEIGHT);
    EXPECT_EQ(result.dimension(1), WIDTH);
    EXPECT_EQ(result.dimension(2), channels);
    EXPECT_TRUE(images_equal(result, image));
}

}  // namespace

TEST(ImageIoTest, WritePngTagsSrgbByDefault) {
    const auto filename = temp_path("image_io_srgb.png");
    write_png(filename, test_image(3));

    const auto types = png_chunk_types(read_file(filename));
    const auto idat = std::find(types.begin(), types.end(), "IDAT");
    ASSERT_NE(idat, types.end());
    for (const char* chunk : {"sRGB", "gAMA", "cHRM"}) {
        EXPECT_NE(std::find(types.begin(), idat, chunk), idat) << chunk;
    }
}

TEST(ImageIoTest, WritePngWithoutSrgb) {
    const auto filename = temp_path("image_io_no_srgb.png");
    PngWriteOptions options;
    options.srgb = false;
    write_png(filename, test_image(3), options);

    const auto types = png_chunk_types(read_file(filename));
    EXPECT_EQ(std::find(types.begin(), types.end(), "sRGB"), types.end());
    EXPECT_EQ(std::find(types.begin(), types.end(), "gAMA"), types.end());
}

TEST(ImageIoTest, PngRoundTrip) {
    for (bool filter_rows : {true, false}) {
        for (Eigen::Index channels = 1; channels <= 4; ++channels) {
            SCOPED_TRACE("channels " + std::to_string(channels) + " filter_rows " +
                         std::to_string(filter_rows));
            expect_png_round_trip(channels, filter_rows);
        }
    }
}

TEST(ImageIoTest, WritePngFromMappedBuffer) {
    const auto filename = temp_path("image_io_mapped.png");
    std::vector<uint8_t> pixels(static_cast<size_t>(WIDTH * HEIGHT * 3));
    for (size_t i = 0; i < pixels.size(); ++i) {
        pixels[i] = static_cast<uint8_t>(i);
    }
    write_png(filename, ImageMap(pixels.data(), HEIGHT, WIDTH, 3));

    const auto result = read_png(filename);
    ASSERT_EQ(result.size(), static_cast<Eigen::Index>(pixels.size()));
    EXPECT_TRUE(std::equal(pixels.begin(), pixels.end(), result.data()));
}

TEST(ImageIoTest, WritePngRejectsInvalidInput) {
    const auto filename = temp_path("image_io_invalid.png");
    const auto pixels = test_image(4);
    EXPECT_THROW(write_png(filename, ImageMap(pixels.data(), 0, 2, 3)), std::invalid_argument);
    EXPECT_THROW(write_png(filename, ImageMap(pixels.data(), 2, 2, 5)), std::invalid_argument);
    EXPECT_THROW(write_png(temp_path("missing_dir/image.png"), ImageMap(pixels.data(), 2, 2, 1)),
                 std::runtime_error);
}

TEST(ImageIoTest, ReadPngRejectsNonPng) {
    const auto filename = temp_path("image_io_not_a.png");
    {
        std::ofstream file(filename, std::ios::binary);
        file << "definitely not a PNG file";
    }
    EXPECT_THROW(read_png(filename), std::runtime_error);
    EXPECT_THROW(read_png(temp_path("image_io_missing.png")), std::runtime_error);
}

TEST(ImageIoTest, GifWriterWritesFrames) {
    const auto filename = temp_path("image_io.gif");
    {
        GifWriter writer(filename, WIDTH, HEIGHT);
        EXPECT_EQ(writer.width(), static_cast<size_t>(WIDTH));
        EXPECT_EQ(writer.height(), static_cast<size_t>(HEIGHT));
        const auto rgb = test_image(3);
        writer.add_frame(rgb, 100.0);
        writer.add_frame(test_image(4), 50.0);
        EXPECT_THROW(writer.add_frame(test_image(1), 50.0), std::invalid_argument);
        EXPECT_THROW(writer.add_frame(test_image(3, WIDTH + 1), 50.0), std::invalid_argument);
        writer.close();
        EXPECT_THROW(writer.add_frame(rgb, 50.0), std::runtime_error);
    }

    const auto data = read_file(filename);
    ASSERT_GT(data.size(), 13u);
    EXPECT_EQ(std::string(data.begin(), data.begin() + 6), "GIF89a");
    EXPECT_EQ(data[6] | (data[7] << 8), WIDTH);
    EXPECT_EQ(data[8] | (data[9] << 8), HEIGHT);
    EXPECT_EQ(data.back(), 0x3B);
}
