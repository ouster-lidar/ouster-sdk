/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster_icon.h"

#include <gtest/gtest.h>

#include <cstddef>

using ouster::sdk::viz::impl::render_ouster_icon;

namespace {

// The Ouster mark is symmetric about both axes; the renderer must preserve that
// exactly (it has regressed on both axes before, from off-by-half sampling).
void expect_symmetric(int size) {
    const auto rgba = render_ouster_icon(size);
    ASSERT_EQ(rgba.size(), static_cast<std::size_t>(size) * size * 4);

    int horizontal_mismatches = 0;
    int vertical_mismatches = 0;
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size / 2; ++x) {
            const std::size_t left = ((static_cast<std::size_t>(y) * size) + x) * 4;
            const std::size_t right = ((static_cast<std::size_t>(y) * size) + (size - 1 - x)) * 4;
            if (rgba[left] != rgba[right]) {
                ++horizontal_mismatches;
            }
        }
    }
    for (int x = 0; x < size; ++x) {
        for (int y = 0; y < size / 2; ++y) {
            const std::size_t top = ((static_cast<std::size_t>(y) * size) + x) * 4;
            const std::size_t bottom = ((static_cast<std::size_t>(size - 1 - y) * size) + x) * 4;
            if (rgba[top] != rgba[bottom]) {
                ++vertical_mismatches;
            }
        }
    }
    EXPECT_EQ(horizontal_mismatches, 0) << "size " << size;
    EXPECT_EQ(vertical_mismatches, 0) << "size " << size;
}

}  // namespace

TEST(OusterIcon, SymmetricAboutBothAxes) {
    expect_symmetric(64);
    expect_symmetric(128);
    expect_symmetric(256);
}

TEST(OusterIcon, LayoutInvariants) {
    // static so the lambda below can read it uncaptured on MSVC (C3493)
    static constexpr int size = 64;
    const auto rgba = render_ouster_icon(size);
    ASSERT_EQ(rgba.size(), static_cast<std::size_t>(size) * size * 4);

    const auto pixel = [](int x, int y) { return ((static_cast<std::size_t>(y) * size) + x) * 4; };

    // Dark ground in the corners, white mark in the center.
    EXPECT_EQ(rgba[pixel(0, 0)], 0);
    EXPECT_EQ(rgba[pixel(size - 1, 0)], 0);
    EXPECT_EQ(rgba[pixel(0, size - 1)], 0);
    EXPECT_EQ(rgba[pixel(size - 1, size - 1)], 0);
    EXPECT_EQ(rgba[pixel(size / 2, size / 2)], 255);

    // Fully opaque.
    for (std::size_t i = 3; i < rgba.size(); i += 4) {
        EXPECT_EQ(rgba[i], 255);
    }
}
