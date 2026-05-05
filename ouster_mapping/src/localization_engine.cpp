#include <ouster/cloud_io.h>
#include <ouster/impl/logging.h>
#include <ouster/localization_engine.h>

#include <chrono>
#include <stdexcept>
#include <string>

#include "kiss_localization.h"

using ouster::sdk::core::logger;

namespace ouster {
namespace sdk {
namespace mapping {

LocalizationEngine::LocalizationEngine(
    const std::vector<std::shared_ptr<core::SensorInfo>>& infos,
    const LocalizationConfig& config, const std::string& map_path)
    : LocalizationEngine(infos, config, load_map(map_path)) {}

LocalizationEngine::LocalizationEngine(
    const std::vector<std::shared_ptr<core::SensorInfo>>& infos,
    const LocalizationConfig& config,
    const Eigen::Ref<const core::PointCloudXYZf> map) {
    if (config.backend != "kiss") {
        throw std::runtime_error(std::string{"Unsupported backend: "} +
                                 config.backend);
    }

    backend_ = std::make_unique<KissLocalization>(infos, config, map);
}

void LocalizationEngine::update(core::LidarScanSet& scans) {
    backend_->update(scans);
}

core::PointCloudXYZf LocalizationEngine::load_map(const std::string& map_file) {
    auto start = std::chrono::steady_clock::now();
    core::PointCloudXYZf points = ouster::sdk::core::read_pointcloud(map_file);
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    logger().info("Took {} seconds to load the map {} which has {} points",
                  elapsed.count(), map_file, points.rows());
    return points;
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
