/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <vector>

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

// Render the Ouster logo procedurally as an 8-bit RGBA image (white mark on a
// black ground, opaque) of `size` x `size` pixels. The mark is defined by a few
// even-quartic edge curves fit to the Ouster SVG; edges are anti-aliased via
// supersampling. No image asset or image-decoding dependency is involved.
std::vector<unsigned char> render_ouster_icon(int size);

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster
