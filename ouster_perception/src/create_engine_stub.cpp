/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include <ouster/core/sensor_info.h>
#include <ouster/perception/detection_engine.h>

#include <memory>
#include <vector>

namespace ouster {
namespace sdk {
namespace perception {

std::unique_ptr<DetectionEngine> DetectionEngine::create(
    const std::vector<std::shared_ptr<core::SensorInfo>>& /*sensor_infos*/,
    const ClassicDetectionConfig& /*config*/) {
    throw std::runtime_error("DetectionEngine is only available with distributed binaries.");
}

std::unique_ptr<DetectionEngine> DetectionEngine::create(
    const std::vector<std::shared_ptr<core::SensorInfo>>& /*sensor_infos*/,
    const std::string& /*kind*/) {
    throw std::runtime_error("DetectionEngine is only available with distributed binaries.");
}

}  // namespace perception
}  // namespace sdk
}  // namespace ouster
