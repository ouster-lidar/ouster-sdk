/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster_icon.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

namespace {

// The Ouster mark is three horizontal "ribbon" bands, symmetric about both axes.
// Each band edge is an even quartic in u = x - 0.5, fit to the Ouster SVG to
// < 0.1 px at icon sizes. Symmetry minimizes the data:
//   * x-symmetry is built into the even powers (no left/right duplication);
//   * y-symmetry means the bottom band is the mirror of the top and the middle
//     band is self-symmetric, so only three edges are needed -- the top band's
//     two edges and the middle band's top edge.
// Coordinates are normalized: x in [0, 1] across the logo width, y in the same
// scale (logo height = ASPECT).
constexpr float ASPECT = 912.0f / 1515.0f;  // logo height / width
constexpr float CENTER_Y = ASPECT * 0.5f;   // horizontal axis of symmetry

// {a0, a2, a4} for y(x) = a0 + a2 * u^2 + a4 * u^4, with u = x - 0.5.
constexpr std::array<float, 3> TOP_OUTER = {0.1340584f, -0.5024997f, -0.1340832f};
constexpr std::array<float, 3> TOP_INNER = {0.2182081f, -0.2510300f, -0.0055932f};
constexpr std::array<float, 3> MID_TOP = {0.2589497f, -0.1233800f, -0.0120105f};

constexpr float MARK_WIDTH_FRAC = 0.76f;  // mark occupies this fraction of the icon
constexpr int SUPERSAMPLE = 4;            // render at Nx, then box-downscale to AA

float eval_edge(const std::array<float, 3>& coeffs, float x) {
    const float off = x - 0.5f;
    const float off_sq = off * off;
    return coeffs[0] + (coeffs[1] * off_sq) + (coeffs[2] * (off_sq * off_sq));
}

// Fill column `col` of the coverage buffer between pixel rows spanning [py0, py1].
void fill_span(std::vector<std::uint8_t>& buf, int dim, int col, float py0, float py1) {
    int top = static_cast<int>(std::lround(std::min(py0, py1)));
    int bottom = static_cast<int>(std::lround(std::max(py0, py1)));
    top = std::max(top, 0);
    bottom = std::min(bottom, dim - 1);
    for (int y = top; y <= bottom; ++y) {
        buf[(static_cast<std::size_t>(y) * dim) + col] = 255;
    }
}

}  // namespace

std::vector<unsigned char> render_ouster_icon(int size) {
    const int big = size * SUPERSAMPLE;
    std::vector<std::uint8_t> hires(static_cast<std::size_t>(big) * big, 0);  // black ground
    const float scale = MARK_WIDTH_FRAC * static_cast<float>(big);
    const float off_x = (static_cast<float>(big) - scale) * 0.5f;
    const float off_y = (static_cast<float>(big) - (ASPECT * scale)) * 0.5f;
    const float two_cy = 2.0f * CENTER_Y;

    for (int col = 0; col < big; ++col) {
        // Sample the pixel *center* so columns are symmetric about x = 0.5;
        // sampling the left edge makes the (even-polynomial) mark subtly lopsided.
        const float x = (static_cast<float>(col) + 0.5f - off_x) / scale;
        if (x < 0.0f || x > 1.0f) {
            continue;
        }
        const float y_outer = eval_edge(TOP_OUTER, x);
        const float y_inner = eval_edge(TOP_INNER, x);
        const float y_mid = eval_edge(MID_TOP, x);
        fill_span(hires, big, col, off_y + (y_outer * scale),
                  off_y + (y_inner * scale));  // top band
        fill_span(hires, big, col, off_y + (y_mid * scale),
                  off_y + ((two_cy - y_mid) * scale));  // middle band
        fill_span(hires, big, col, off_y + ((two_cy - y_inner) * scale),
                  off_y + ((two_cy - y_outer) * scale));  // bottom band
    }

    // Enforce exact vertical symmetry (the mark mirrors about y = 0.5): copy the
    // top half onto the bottom, eliminating any half-pixel rounding asymmetry.
    for (int y = 0; y < big / 2; ++y) {
        const std::size_t top = static_cast<std::size_t>(y) * big;
        const std::size_t bottom = static_cast<std::size_t>(big - 1 - y) * big;
        for (int x = 0; x < big; ++x) {
            hires[bottom + x] = hires[top + x];
        }
    }

    // Box-downscale the supersampled coverage to grayscale, expand to opaque RGBA.
    std::vector<unsigned char> rgba(static_cast<std::size_t>(size) * size * 4);
    const int area = SUPERSAMPLE * SUPERSAMPLE;
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            int sum = 0;
            for (int dy = 0; dy < SUPERSAMPLE; ++dy) {
                const std::size_t row = static_cast<std::size_t>((y * SUPERSAMPLE) + dy) * big;
                for (int dx = 0; dx < SUPERSAMPLE; ++dx) {
                    sum += hires[row + static_cast<std::size_t>((x * SUPERSAMPLE) + dx)];
                }
            }
            const auto gray = static_cast<unsigned char>(sum / area);
            const std::size_t out = ((static_cast<std::size_t>(y) * size) + x) * 4;
            rgba[out] = gray;
            rgba[out + 1] = gray;
            rgba[out + 2] = gray;
            rgba[out + 3] = 255;
        }
    }
    return rgba;
}

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster
