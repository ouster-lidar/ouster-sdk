#include <iostream>
#include <string>
#include <vector>

#include "ouster/core/frame_set.h"
#include "ouster/core/open_source.h"
#include "ouster/sensor/sensor_frame_set_source.h"

using namespace ouster::sdk;

namespace {

void iterate_source(core::FrameSetSource& source, std::size_t limit = 5) {
    std::size_t count = 0;
    // clang-format off
    // [doc-stag-multi-sensorframesetsource-loop]
    for (const auto& frame_set : source) {
        for (std::size_t idx = 0; idx < frame_set.size();++idx) {
            const auto& frame = frame_set[idx];
            if (frame) {
                std::cout << "frame: " << count
                          << " id=" << frame->frame_id << "\n";
                // [doc-etag-multi-sensorframesetsource-loop]
                // clang-format on
            } else {
                continue;
            }
        }
        if (++count >= limit) {
            break;
        }
    }
}

}  // namespace

void stream_multi_sensor(const std::vector<std::string>& hostnames, std::size_t limit = 5) {
    // [doc-stag-multi-sensorframesetsource]
    // Replace with actual hostnames
    // std::vector<std::string> hostnames{"os-xxxx.local", "os-yyyy.local"};
    sensor::SensorFrameSetSource source(hostnames);
    // [doc-etag-multi-sensorframesetsource]
    iterate_source(source, limit);
}

void stream_multi_open_source(const std::vector<std::string>& hostnames, std::size_t limit = 5) {
    // clang-format off
    // [doc-stag-multi-opensource-sensor]
    // Replace with actual hostnames
    // auto hostnames = {"os-xxxx.local", "os-yyyy.local"};
    auto source = open_source(hostnames);
    //! [doc-etag-multi-opensource-sensor]
    std::size_t count = 0;
    for (const auto& frame_set : source) {
        for (std::size_t idx = 0; idx < frame_set.size();++idx) {
            const auto& frame = frame_set[idx];
            if (frame) {
                std::cout << "frame: " << count
                          << " id=" << frame->frame_id << "\n";
            } }
        if (++count >= limit) { break; } }
    // clang-format on
}

void stream_multi_open_source(const std::vector<std::string>& hostnames,
                              const std::vector<std::string>& metadata_paths,
                              std::size_t limit = 5) {
    auto source =
        open_source(hostnames, [&](FrameSetSourceOptions& opt) { opt.meta = metadata_paths; });
    iterate_source(source, limit);
}

void replay_multi_recording(const std::vector<std::string>& paths, std::size_t limit = 5) {
    // clang-format off
    // [doc-stag-multi-opensource-file]
    // auto paths = {"capture.pcap", "capture2.pcap"};
    auto source = open_source(paths);

    std::size_t count = 0;
    for (const auto& frame_set : source) {
        for (std::size_t idx = 0; idx < frame_set.size();++idx) {
            const auto& frame = frame_set[idx];
            if (frame) {
                std::cout << "frame: " << count
                          << " id=" << frame->frame_id << "\n";
            } else {
                continue;
            }
        }
        if (++count >= limit) {
            break;
        }
    }
    // [doc-etag-multi-opensource-file]
    // clang-format on
}
