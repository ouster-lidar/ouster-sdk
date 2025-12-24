#include <ouster/slam_engine.h>

#include <stdexcept>

#include "kiss_slam.h"

namespace ouster {
namespace sdk {
namespace mapping {

SlamEngine::SlamEngine(
    const std::vector<std::shared_ptr<core::SensorInfo>>& infos,
    const SlamConfig& config) {
    if (config.backend != "kiss") {
        throw std::runtime_error(std::string{"Unsupported backend: "} +
                                 config.backend);
    }

    backend_ = std::make_unique<KissSlam>(infos, config);
}

void SlamEngine::update(core::LidarScanSet& scans) { backend_->update(scans); }

core::PointCloudXYZf SlamEngine::get_point_cloud() const {
    return backend_->get_point_cloud();
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
