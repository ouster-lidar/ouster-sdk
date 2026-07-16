#include <iostream>

#include "ouster/core/open_source.h"
#ifdef OUSTER_OSF
#include "ouster/osf/osf_frame_set_source.h"
#endif
#ifdef OUSTER_PCAP
#include "ouster/pcap/pcap_frame_set_source.h"
#endif
#ifdef OUSTER_SENSOR
#include "ouster/sensor/sensor_frame_set_source.h"
#endif
#include "ouster/perception/detection_engine.h"
#include "ouster/perception/object.h"
// NOLINTBEGIN(google-build-using-namespace)
using namespace ouster::sdk;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " [source]\n";
        std::exit(1);
    }
    auto* source_url = argv[1];
    // clang-format off
    // [doc-stag-perception-engine]
    // Open a frame set source
    auto source = open_source(source_url);

    // Create a detection engine
    auto engine = perception::DetectionEngine::create(source.sensor_info());

    // Iterate over the frame sets in the frame set source
    int frame_counter{0};
    for (auto frame_set : source) {
        auto& frame = *frame_set[0];

        // Apply the engine to the frame (or the whole frame set.)
        engine->update(frame);

        // Inspect the objects added by the engine.
        std::cerr << "Frame: " << frame_counter << '\n';
        frame_counter++;
        for (const auto& kv : frame.objects()) {
            std::cerr << "Key " << kv.first << " has " << kv.second.size() << " objects.\n";
        }
    }
    // [doc-etag-perception-engine]
    // clang-format on
}
// NOLINTEND(google-build-using-namespace)