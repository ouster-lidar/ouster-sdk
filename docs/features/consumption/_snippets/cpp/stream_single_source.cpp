#include <iostream>
#include <string>
#include <vector>

#include "ouster/osf/osf_frame_set_source.h"
#include "ouster/sensor/sensor_frame_set_source.h"

//! [doc-stag-opensource-imports]
#include "ouster/core/open_source.h"
using namespace ouster::sdk;
//! [doc-etag-opensource-imports]

//! [doc-stag-pcapframesetsource-imports]
#include "ouster/pcap/pcap_frame_set_source.h"
//! [doc-etag-pcapframesetsource-imports]

namespace {

void print_frames(core::FrameSetSource& source, std::size_t limit = 10) {
    std::size_t count = 0;
    for (const auto& frame_set : source) {
        for (const auto& frame_ptr : frame_set) {
            if (!frame_ptr) {
                continue;
            }
            std::cout << "frame " << count << ": id=" << frame_ptr->frame_id << '\n';
            if (++count >= limit) {
                return;
            }
        }
    }
}

}  // namespace

void stream_live_sensor(const std::string& hostname) {
    // [doc-stag-single-sensorframesetsource]
    sensor::SensorFrameSetSource source(hostname);
    // [doc-etag-single-sensorframesetsource]
    print_frames(source);
}

void stream_live_via_open_source_collate(const std::string& hostname) {
    // [doc-stag-single-opensource]
    auto source = open_source(hostname);
    // [doc-etag-single-opensource]
    print_frames(source);
}

void close_open_source(const std::string& hostname) {
    // [doc-stag-single-opensource-close]
    {
        auto source = open_source(hostname, {}, /*collate=*/true, /*sensor_idx=*/0);
        for (const auto& frame_set : source) {
            if (frame_set.size() > 0 && frame_set[0]) {
                std::cout << "first frame_id=" << frame_set[0]->frame_id << "\n";
                break;
            }
        }
    }
    // Underlying resources (UDP sockets for live sources, file handles for
    // replay) are released when source is destroyed at end of scope.
    // [doc-etag-single-opensource-close]
}

void select_single_sensor(const std::string& source_url) {
    // [doc-stag-single-select-equivalent]
    // [doc-stag-single-opensource-nocollate]
    // Shortcut: select sensor 0 while opening. Note: sensor_idx >= 0 makes
    // open_source ignore collate entirely, so its value here has no effect.
    auto via_open_source = open_source(source_url, {}, /*collate=*/false, /*sensor_idx=*/0);
    // [doc-etag-single-opensource-nocollate]

    // Equivalent: open uncollated, then derive a single-sensor view.
    auto via_single = open_source(source_url, {}, /*collate=*/false).single(0);
    // [doc-etag-single-select-equivalent]
    // [doc-stag-single-opensource-nocollate-loop]
    // Both are Singler-wrapped, single-sensor sources, so both narrow every
    // FrameSet to one entry, accessible the same way: frame_set[0].
    for (const auto& frame_set : via_open_source) {
        if (frame_set.size() > 0 && frame_set[0]) {
            std::cout << "via sensor_idx: frame_id=" << frame_set[0]->frame_id << "\n";
            break;
        }
    }
    for (const auto& frame_set : via_single) {
        if (frame_set.size() > 0 && frame_set[0]) {
            std::cout << "via single():   frame_id=" << frame_set[0]->frame_id << "\n";
            break;
        }
    }
    // [doc-etag-single-opensource-nocollate-loop]
}

void replay_data(const std::string& data_path, const std::string& metadata_path) {
    {
        // [doc-stag-pcap-replay]
        auto source = open_source(data_path);
        // [doc-etag-pcap-replay]
        print_frames(source);
    }

    {
        // [doc-stag-pcapframesetsource]
        pcap::PcapFrameSetSourceOptions options;
        options.meta = std::vector<std::string>{metadata_path};
        auto source = pcap::PcapFrameSetSource(data_path, options);
        // [doc-etag-pcapframesetsource]
        print_frames(source);
    }
}

void replay_pcap_metadata(const std::string& pcap_path, const std::string& metadata_path) {
    // [doc-stag-pcapframesetsource-metadata]
    pcap::PcapFrameSetSourceOptions options;
    options.meta = {metadata_path};
    auto source = pcap::PcapFrameSetSource(pcap_path, options);
    // [doc-etag-pcapframesetsource-metadata]
    print_frames(source);
}

void replay_open_source_metadata(const std::string& pcap_path,
                                 const std::string& sensor_info_path) {
    // clang-format off
    // [doc-stag-pcap-replay-metadata]
    auto source = open_source(pcap_path, [&](FrameSetSourceOptions& opt) { 
                              opt.meta = { sensor_info_path }; });
    // [doc-etag-pcap-replay-metadata]
    // clang-format on
    print_frames(source);
}

void read_osf_frames(const std::string& osf_path, std::size_t limit) {
    // clang-format off
    // [doc-stag-osf-read-frames]
    auto source = osf::OsfFrameSetSource(osf_path);
    std::size_t count = 0;

    for (const auto& frame_set : source) {
        for (const auto& frame : frame_set) {
            if (!frame) { 
                continue; }
            std::cout << "frame = " << to_string(*frame)
                      << ", WxH=" << frame->w << "x" << frame->h << '\n';
            if (++count >= limit) {
                return;
    } } }
    // [doc-etag-osf-read-frames]
    // clang-format on
}
