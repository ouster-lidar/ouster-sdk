#include <ouster/mapping/slam_engine.h>

#include <memory>
#include <vector>

#include "lio_slam.h"
#include "ouster/core/sensor_info.h"

namespace ouster {
namespace sdk {
namespace mapping {

std::unique_ptr<SlamEngine> SlamEngine::create(
    const std::vector<std::shared_ptr<core::SensorInfo>>& sensor_infos,
    const LIOSlamConfig& config) {
    return std::make_unique<LIOSlam>(sensor_infos, config);
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
