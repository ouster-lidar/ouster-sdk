#include <string>
#include <vector>

#include "ouster/algorithm/align_clouds.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/open_source.h"
#include "ouster/core/types.h"

using namespace ouster::sdk;
using ouster::sdk::algorithm::align_clouds;

core::Matrix4dR align_pairwise_example(const std::string& source_file) {
    //! [doc-stag-alignment-pairwise]
    auto source = open_source(source_file);
    core::FrameSet frames = *source.begin();
    const auto& target_frame = frames[0];
    const auto& source_frame = frames[1];
    auto source_to_target_transform = align_clouds(*source_frame, *target_frame);
    auto aligned_source_extrinsic =
        source_to_target_transform * source_frame->sensor_info->sensor_to_body;
    //! [doc-etag-alignment-pairwise]

    return aligned_source_extrinsic;
}

std::vector<core::Matrix4dR> align_frame_set_example(const std::string& source_file) {
    //! [doc-stag-alignment-frame-set]
    auto source = open_source(source_file);
    core::FrameSet frames = *source.begin();
    auto aligned_extrinsics = align_clouds(frames);

    for (std::size_t i = 0; i < aligned_extrinsics.size(); ++i) {
        source.sensor_info()[i]->sensor_to_body = aligned_extrinsics[i];
    }
    //! [doc-etag-alignment-frame-set]
    // clang-format on

    return aligned_extrinsics;
}
