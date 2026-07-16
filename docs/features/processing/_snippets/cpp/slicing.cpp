/*
 * Copyright (c) Ouster, Inc.
 * All rights reserved.
 *
 * Documentation snippets for the processing guide (C++ slicing examples).
 */

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ouster/core/frame_set_source.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/open_source.h"
#include "ouster/core/xyzlut.h"

//! [doc-stag-slicing-imports]
using namespace ouster::sdk;
//! [doc-etag-slicing-imports]

namespace ouster {
namespace docs {

core::AnyFrameSetSource open_indexed_source_example_cpp(
    const std::string& pcap_path = "<SAMPLE_DATA_PCAP_PATH>") {
    // clang-format off
    //! [doc-stag-slicing-open-source]
    auto source = sdk::open_source(pcap_path, [](sdk::FrameSetSourceOptions& opts) { 
                                   opts.index = true; });
    //! [doc-etag-slicing-open-source]
    // clang-format on
    return source;
}

void print_nth_frame_example_cpp(const core::FrameSetSource& source) {
    if (!source.is_indexed()) {
        throw std::runtime_error("Random access requires an indexed FrameSetSource.");
    }
    // clang-format off
    //! [doc-stag-slicing-print-nth]
    auto frame_set = source[9];
    for (auto& frame : frame_set) {
        if (frame) {
            std::cout << frame->frame_id << '\n'; } }
    // To directly access the 9th frame from sensor index 0 without looping.
    auto frame = source[9][0];
    std::cout << frame->frame_id << '\n';
    //! [doc-etag-slicing-print-nth]
    // clang-format on
}

void print_last_frame_example_cpp(const core::FrameSetSource& source) {
    if (!source.is_indexed()) {
        throw std::runtime_error("Random access requires an indexed FrameSetSource.");
    }
    // clang-format off
    //! [doc-stag-slicing-print-last]
    auto frame = source[-1][0];
    std::cout << frame->frame_id << '\n';
    //! [doc-etag-slicing-print-last]
    // clang-format on
}

void print_first_n_frames_example_cpp(const core::FrameSetSource& source) {
    if (!source.is_indexed()) {
        throw std::runtime_error("Random access requires an indexed FrameSetSource.");
    }
    // clang-format off
    //! [doc-stag-slicing-first-n]
    for (const auto& frame_set : source[{0, 9}]) {
        for (auto& frame : frame_set) {
            if (frame) { 
                std::cout << frame->frame_id << '\n'; } } }
    //! [doc-etag-slicing-first-n]
    // clang-format on
}

void print_last_n_frames_example_cpp(const core::FrameSetSource& source) {
    if (!source.is_indexed()) {
        throw std::runtime_error("Random access requires an indexed FrameSetSource.");
    }
    // clang-format off
    //! [doc-stag-slicing-last-n]
    for (const auto& frame_set : source[{-9, -1}]) {
        for (const auto& frame : frame_set) {
            if (frame) { 
                std::cout << frame->frame_id << '\n'; } } }
    //! [doc-etag-slicing-last-n]
    // clang-format on
}

void print_step_sliced_frames_example_cpp(const core::FrameSetSource& source) {
    if (!source.is_indexed()) {
        throw std::runtime_error("Random access requires an indexed FrameSetSource.");
    }
    // clang-format off
    //! [doc-stag-slicing-step]
    // prints every second frame in the first ten
    for (const auto& frame_set : source[{0, 10, 2}]) {
        for (const auto& frame : frame_set) {
            if (frame) { std::cout << frame->frame_id << '\n'; } 
        } }
    //! [doc-etag-slicing-step]
    // clang-format on
}

void slicing_as_source_example_cpp(const core::FrameSetSource& source) {
    if (!source.is_indexed()) {
        throw std::runtime_error("Random access requires an indexed FrameSetSource.");
    }
    // clang-format off
    //! [doc-stag-slicing-subsource]
    auto source2 = source[{5, 10}];
    // This should print 5 since source2 is scoped to the range [5, 10]
    std::cout << "source2 length: " << source2.size() << '\n';
    // This is equivalent to calling source[5][0]
    const auto first_frame = source2[0][0];
    std::cout << first_frame->frame_id << '\n';
    // This is equivalent to calling source[9][0]
    const auto fifth_frame = source2[4][0];
    std::cout << fifth_frame->frame_id << '\n';

    // Use source2 as an iterator similar to the main source, assumes only one sensor
    for (const auto& frame_set : source2) {
        if (frame_set.size() > 0 && frame_set[0]) {
            std::cout << frame_set[0]->frame_id << '\n'; } }
    
    // it is possible to sub slice, meaning take the result of a previous slice operation and slice it
    // Thus, the following statement is valid
    auto source3 = source2[{2, 4}];
    std::cout << "source3 length: " << source3.size() << '\n';  // equivalent to source[7:9]
    //! [doc-etag-slicing-subsource]
    // clang-format on
}

void first_valid_column_example_cpp(const core::FrameSetSource& source) {
    if (!source.is_indexed()) {
        throw std::runtime_error("Random access requires an indexed FrameSetSource.");
    }
    // clang-format off
    //! [doc-stag-slicing-first-valid-column]
    auto frame = source[0][0]; 
    std::cout << frame->get_first_valid_column() << '\n';
    //! [doc-etag-slicing-first-valid-column]
    // clang-format on
}

void last_valid_column_example_cpp(const core::FrameSetSource& source) {
    if (!source.is_indexed()) {
        throw std::runtime_error("Random access requires an indexed FrameSetSource.");
    }
    // clang-format off
    //! [doc-stag-slicing-last-valid-column]
    auto frame = source[0][0]; 
    std::cout << frame->get_last_valid_column() << '\n';
    //! [doc-etag-slicing-last-valid-column]
    // clang-format on
}

}  // namespace docs
}  // namespace ouster
