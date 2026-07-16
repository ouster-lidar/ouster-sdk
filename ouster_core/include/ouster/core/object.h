/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once
#include <Eigen/Dense>
#include <string>
#include <unordered_map>

#include "ouster/core/pose.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

using UnalignedVector3f = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;

/**
 * Data holder class for perception annotations
 */
class OUSTER_API_CLASS Object {
   public:
    OUSTER_API_FUNCTION
    Object() = default;

    /**
     * Object ID.
     *
     * If tracked, objects maintain their IDs between frames.
     */
    uint32_t id{};

    /**
     * Timestamp of the first detection, in nanoseconds. Lidar time.
     */
    uint64_t creation_ts{};

    /**
     * Timestamp of the current detection, in nanoseconds. Lidar time.
     */
    uint64_t timestamp{};

    /**
     * Object classification.
     *
     * DetectionEngine implementations determine the meaning behind class_id
     * numbers, please refer to @ref ouster::sdk::core::ClassMap to get
     * corresponding class name strings.
     */
    uint32_t class_id{};

    /**
     * Classification confidence.
     *
     * Value between 0 and 1, where 1 is absolute confidence.
     */
    float class_confidence{0.0f};

    /**
     * Transform from the object frame to the body frame.
     */
    Pose object_to_body{};

    /**
     * Transform from the body frame to the world frame.
     */
    Pose body_to_world{};

    /**
     * Velocity vector (inside worldframe).
     */
    UnalignedVector3f velocity{UnalignedVector3f::Zero()};

    /**
     * Full extents of the object's bounding box.
     */
    UnalignedVector3f dimensions{UnalignedVector3f::Zero()};

    /**
     * Properties dictionary.
     *
     * Allows to extend object information with arbitrary data.
     */
    std::unordered_map<std::string, std::string> properties;

    /**
     * Equality for Objects.
     *
     * @param[in] other another object to compare to.
     *
     * @return true if equal
     */
    OUSTER_API_FUNCTION
    bool operator==(const Object& other) const;
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
