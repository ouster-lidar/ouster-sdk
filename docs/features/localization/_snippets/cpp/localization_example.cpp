#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

//! [doc-stag-localization-imports]
#include "ouster/core/open_source.h"
#include "ouster/core/pose_conversion.h"
#include "ouster/core/types.h"
#include "ouster/mapping/localization_engine.h"
using namespace ouster::sdk;
//! [doc-etag-localization-imports]

#ifdef OUSTER_OSF
#include "ouster/osf/osf_frame_set_source.h"
#endif

// clang-format off
int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <source> <map.ply>" << std::endl;
        return 1;
    }
    std::string source_path = argv[1];
    std::string map_file    = argv[2];

    //! [doc-stag-localization-config]
    mapping::LIOLocalizationConfig config;
    config.min_range = 0.5;
    config.max_range = 100.0;
    config.voxel_size = 1.0;
    config.deskew_method = "auto";
    //! [doc-etag-localization-config]

    //! [doc-stag-localization-loop]
    //! [doc-stag-localization-engine]
    auto source = open_source(source_path);
    auto engine = mapping::LocalizationEngine::create(
        /*sensor_infos=*/source.sensor_info(),
        /*map_path=*/map_file,
        /*config=*/config);
    //! [doc-etag-localization-engine]
    //! [doc-stag-localization-loop-update]
    for (auto frame_set : source) {
        engine->update(frame_set);
        //! [doc-etag-localization-loop-update]
        //! [doc-stag-localization-loop-printpose]
        for (const auto& frame : frame_set.valid_frames()) {
            auto col = frame->get_last_valid_column();
            auto frame_pose = frame->get_column_pose(col);
            auto frame_ts = frame->timestamp()[col];
            Eigen::Vector3d t = frame_pose.block<3, 1>(0, 3);
            Eigen::Matrix3d rot = frame_pose.block<3, 3>(0, 0);
            auto angles = core::matrix_to_euler(rot);  // [roll, pitch, yaw] in radians
            constexpr double rad2deg = 180.0 / M_PI;
            auto roll  = angles(0) * rad2deg;
            auto pitch = angles(1) * rad2deg;
            auto yaw   = angles(2) * rad2deg;

            std::cout << std::fixed
                      << "idx = " << frame->frame_id << "; ts = " << frame_ts << "; "
                      << "XYZ: " << std::setprecision(2) << t(0) << ", " << t(1) << ", " << t(2) << " "
                      << "(R: " << std::setprecision(1) << roll << ", P: " << pitch << ", Y: " << yaw << ")"
                      << std::endl;
        }
        //! [doc-etag-localization-loop-printpose]
    }
    //! [doc-etag-localization-loop]

    return 0;
}
// clang-format on
