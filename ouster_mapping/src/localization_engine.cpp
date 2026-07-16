#include <ouster/core/cloud_io.h>
#include <ouster/core/impl/logging.h>
#include <ouster/mapping/localization_engine.h>

#include <chrono>
#include <stdexcept>
#include <string>

#include "lio_localization.h"

using ouster::sdk::core::logger;

namespace ouster {
namespace sdk {
namespace mapping {

std::unique_ptr<LocalizationEngine> LocalizationEngine::create(
    const std::vector<std::shared_ptr<core::SensorInfo>>& infos, const std::string& map_path,
    const LIOLocalizationConfig& config) {
    return create(infos, load_map(map_path), config);
}

std::unique_ptr<LocalizationEngine> LocalizationEngine::create(
    const std::vector<std::shared_ptr<core::SensorInfo>>& infos,
    const Eigen::Ref<const core::PointCloudXYZf> map, const LIOLocalizationConfig& config) {
    return std::make_unique<LIOLocalization>(infos, config, map);
}

LocalizationEngine::LocalizationEngine(const std::vector<std::shared_ptr<core::SensorInfo>>& infos)
    : infos_(infos) {}

core::PointCloudXYZf LocalizationEngine::load_map(const std::string& map_file) {
    auto start = std::chrono::steady_clock::now();
    core::PointCloudXYZf points = ouster::sdk::core::read_pointcloud(map_file);
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    logger().info("Took {} seconds to load the map {} which has {} points", elapsed.count(),
                  map_file, points.rows());
    return points;
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
