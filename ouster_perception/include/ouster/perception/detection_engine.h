/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>

#include "ouster/core/frame_set.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace perception {

/**
 * Base detection configuration shared by detection engine kinds
 */
struct OUSTER_API_CLASS DetectionConfig {
    OUSTER_API_FUNCTION
    virtual ~DetectionConfig() = default;

    /**
     * @brief Determines whether the engine will save instance id pixel fields
     */
    bool save_instance_id_fields = true;

    /**
     * Downcast to child config
     *
     * @return config as child type
     */
    template <typename ChildT>
    ChildT& as() {
        return dynamic_cast<ChildT&>(*this);
    }

    /**
     * Downcast to child config
     *
     * @return config as child type
     */
    template <typename ChildT>
    const ChildT& as() const {
        return dynamic_cast<const ChildT&>(*this);
    }
};

/**
 * Config for classic detection engine
 */
struct OUSTER_API_CLASS ClassicDetectionConfig : DetectionConfig {
    OUSTER_API_FUNCTION
    ~ClassicDetectionConfig() override = default;

    /**
     * @brief The minimum side length that a cluster must have in order
     *        to be kept
     */
    float cluster_filter_min_side_length{0.3f};

    /**
     * @brief The maximum side length that a cluster must have in order
     *        to be kept (ignored when value is negative)
     */
    float cluster_filter_max_side_length{10.0f};

    /**
     * @brief The minimum size a cluster may be along the vertical dimension.
     *        Allows filtering out of very flat objects that may be outliers.
     */
    float cluster_filter_min_vertical_size{1.0f};

    /**
     * @brief The maximum volume a cluster can have in order to be kept
     *        (ignored when value is negative)
     */
    float cluster_filter_max_volume{45.0f};
};

/**
 * Provides interface for extracting objects from sensor data
 */
class OUSTER_API_CLASS DetectionEngine {
   public:
    OUSTER_API_FUNCTION
    virtual ~DetectionEngine() = default;

    /**
     * Get a map of class_id to class_name
     *
     * @return map of ids to names
     */
    OUSTER_API_FUNCTION
    virtual const std::unordered_map<uint64_t, std::string>& class_map() const = 0;

    /**
     * Detect objects in a LidarFrame.
     * Objects will be saved into the frame.
     *
     * @param[in] frame lidar frame
     */
    OUSTER_API_FUNCTION
    virtual void update(core::LidarFrame& frame) = 0;

    /**
     * Detect objects in a FrameSet.
     * Objects will be saved into the frame_set
     *
     * @param[in] frame_set collation of lidar frames
     */
    OUSTER_API_FUNCTION
    virtual void update(core::FrameSet& frame_set) = 0;

    /**
     * Creates and initializes classic detection engine with default config.
     *
     * @param[in] sensor_infos a vector of SensorInfo pointers
     * @param[in] config config for the classic engine (defaults to
     *            ClassicDetectionConfig{})
     * @return unique_ptr to detection engine
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<DetectionEngine> create(
        const std::vector<std::shared_ptr<core::SensorInfo>>& sensor_infos,
        const ClassicDetectionConfig& config = ClassicDetectionConfig{});

    /**
     * Creates and initializes a detection engine from a kind string.
     *
     * Currently only "classic" is supported.
     *
     * @param[in] sensor_infos a vector of SensorInfo pointers
     * @param[in] kind backend identifier (e.g. "classic")
     * @return unique_ptr to detection engine
     * @throws std::invalid_argument if kind is not recognised
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<DetectionEngine> create(
        const std::vector<std::shared_ptr<core::SensorInfo>>& sensor_infos,
        const std::string& kind);
};

}  // namespace perception
}  // namespace sdk
}  // namespace ouster
