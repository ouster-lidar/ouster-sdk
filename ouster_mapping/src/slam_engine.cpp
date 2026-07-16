#include <ouster/mapping/slam_engine.h>

#include <stdexcept>

#include "lio_slam.h"

namespace ouster {
namespace sdk {
namespace mapping {

std::unique_ptr<SlamEngine> SlamEngine::create(
    const std::vector<std::shared_ptr<core::SensorInfo>>& infos, const LIOSlamConfig& config) {
    return std::make_unique<LIOSlam>(infos, config);
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
