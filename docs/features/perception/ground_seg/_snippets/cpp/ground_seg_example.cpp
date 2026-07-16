#include <cstdint>
#include <string>
#include <vector>

#include "ouster/algorithm/ground_seg.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/open_source.h"
#include "ouster/core/types.h"
#include "ouster/osf/osf_frame_set_source.h"

using namespace ouster::sdk;
using ouster::sdk::algorithm::GroundSegEngine;

core::img_t<uint8_t> segment_ground(const std::string& source_file,
                                    const std::vector<core::Matrix4dR>& plumb_extrinsics) {
    //! [doc-stag-ground-seg-api]
    auto source = open_source(source_file);
    for (std::size_t i = 0; i < source.sensor_info().size(); ++i) {
        source.sensor_info()[i]->sensor_to_body = plumb_extrinsics[i];
    }

    auto frames = *source.begin();
    auto engine = GroundSegEngine::create();

    engine->update(frames);
    auto ground_mask = frames[0]->field<uint8_t>(core::ChanField::GROUND);
    //! [doc-etag-ground-seg-api]

    return ground_mask;
}
