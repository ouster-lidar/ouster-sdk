// clang-format off

#include <Eigen/Dense>
#include <memory>
#include <string>

#include <cstdlib>
#include <fstream>

#include "../../../cpp_test_utils/test_utils.h"
//! [doc-stag-po-imports]
#include <ouster/mapping/pose_optimizer.h>
//! [doc-etag-po-imports]


namespace ouster {
namespace sdk {
namespace docs {

using docs_test_utils::env_or_empty;
using docs_test_utils::file_exists;
using docs_test_utils::join_path;

std::string test_data_root() { return env_or_empty("TEST_DATA_DIR"); }

std::string osf_fixture_path() {
    auto osf_path = join_path(join_path(test_data_root(), "mapping"), "loop.osf");
    if (!file_exists(osf_path)) {
        throw std::runtime_error("Required test fixture not found: " +
                                 osf_path);
    }
    return osf_path;
}

void run_pose_optimizer_example() {
    const auto osf_path = osf_fixture_path();
    if (!file_exists(osf_path)) {
        throw std::runtime_error("Required test fixture not found: " + osf_path);
    }
    //! [doc-stag-po-construct]
    ouster::sdk::mapping::PoseOptimizer po(
        /*osf_filename=*/osf_path,
        /*key_frame_distance=*/2.0
    );
    //! [doc-etag-po-construct]

    //! [doc-stag-po-pose-to-pose]
    auto pose_to_pose_constraint =
        std::make_unique<ouster::sdk::mapping::PoseToPoseConstraint>(
        /*timestamp1=*/1765338059263057992,
        /*timestamp2=*/1765338723963851640,
        /*relative_pose=*/ouster::sdk::core::Matrix4dR::Identity(),
        /*rotation_weight=*/1.0,
        /*translation_weights=*/Eigen::Array3d(1.0, 1.0, 1.0)
    );
    po.add_constraint(std::move(pose_to_pose_constraint));
    //! [doc-etag-po-pose-to-pose]

    //! [doc-stag-po-auto-loop]
    auto loop_constraints_added = po.add_relative_loop_constraints(
        /*min_distance_m=*/50.0,
        /*cell_size_m=*/2.0,
        /*icp_score_threshold=*/0.6
    );
    //! [doc-etag-po-auto-loop]

    (void)loop_constraints_added;

    //! [doc-stag-po-point-to-point]
    auto point_to_point_constraint =
        std::make_unique<ouster::sdk::mapping::PointToPointConstraint>(
            /*timestamp1=*/1765338003762995576, /*row1=*/27, /*col1=*/1894,
            /*return_idx1=*/1, /*timestamp2=*/1765338746963563448,
            /*row2=*/29, /*col2=*/1221, /*return_idx2=*/1,
            /*translation_weights=*/Eigen::Array3d(0.2, 0.2, 0.2));
    po.add_constraint(std::move(point_to_point_constraint));
    //! [doc-etag-po-point-to-point]

    //! [doc-stag-po-absolute-point]
    auto absolute_point_constraint =
        std::make_unique<ouster::sdk::mapping::AbsolutePointConstraint>(
        /*timestamp=*/1765338003762995576, /*row=*/27, /*col=*/1894,
        /*return_idx=*/1,
        /*absolute_position=*/Eigen::Vector3d(40.0, 30.0, 10.0),
        /*translation_weights=*/Eigen::Array3d(1.0, 1.0, 1.0)
    );
    po.add_constraint(std::move(absolute_point_constraint));
    //! [doc-etag-po-absolute-point]

    //! [doc-stag-po-absolute-pose]
    ouster::sdk::core::Matrix4dR abs_pose =
        ouster::sdk::core::Matrix4dR::Identity();
    abs_pose.block<3, 1>(0, 3) = Eigen::Vector3d(40.0, 30.0, 10.0);
    auto absolute_pose_constraint =
        std::make_unique<ouster::sdk::mapping::AbsolutePoseConstraint>(
        /*timestamp=*/1765338003762995576,
        /*pose=*/abs_pose, /*rotation_weight=*/1.0,
        /*translation_weights=*/Eigen::Array3d(1.0, 1.0, 1.0));
    auto absolute_pose_constraint_id = po.add_constraint(std::move(absolute_pose_constraint));
    //! [doc-etag-po-absolute-pose]

    //! [doc-stag-po-remove-constraint]
    po.remove_constraint(absolute_pose_constraint_id);
    //! [doc-etag-po-remove-constraint]

    //! [doc-stag-po-solve]
    po.solve();
    //! [doc-etag-po-solve]

    //! [doc-stag-po-save-trajectory]
    auto ts = po.get_timestamps(ouster::sdk::mapping::SamplingMode::COLUMNS);
    auto poses = po.get_poses(ouster::sdk::mapping::SamplingMode::COLUMNS);
    ouster::sdk::mapping::save_trajectory("loop_test_traj.csv", ts, poses);
    //! [doc-etag-po-save-trajectory]

    //! [doc-stag-po-save-osf]
    po.save("po_output.osf");
    //! [doc-etag-po-save-osf]
    //! [doc-etag-po-example]
}

}  // namespace docs
}  // namespace sdk
}  // namespace ouster

int main() {
    try {
        ouster::sdk::docs::run_pose_optimizer_example();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}

// clang-format on
