#include <ouster/core/cloud_io.h>
#include <ouster/core/impl/logging.h>
#include <ouster/core/sensor_info.h>
#include <ouster/core/typedefs.h>
#include <ouster/mapping/localization_engine.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "lio_localization.h"

using ouster::sdk::core::logger;

namespace ouster {
namespace sdk {
namespace mapping {

std::unique_ptr<LocalizationEngine> LocalizationEngine::create(
    const std::vector<std::shared_ptr<core::SensorInfo>>& sensor_infos, const std::string& map_path,
    const LIOLocalizationConfig& config) {
    return create(sensor_infos, load_map(map_path), config);
}

std::unique_ptr<LocalizationEngine> LocalizationEngine::create(
    const std::vector<std::shared_ptr<core::SensorInfo>>& sensor_infos,
    const Eigen::Ref<const core::PointCloudXYZf> map, const LIOLocalizationConfig& config) {
    return std::make_unique<LIOLocalization>(sensor_infos, config, map);
}

LocalizationEngine::LocalizationEngine(
    const std::vector<std::shared_ptr<core::SensorInfo>>& sensor_infos)
    : sensor_infos_(sensor_infos) {}

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
