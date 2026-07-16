/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/object.h"

namespace ouster {
namespace sdk {
namespace core {

bool Object::operator==(const Object& other) const {
    // clang-format off
    return id == other.id &&
           creation_ts == other.creation_ts &&
           timestamp == other.timestamp &&
           class_id == other.class_id &&
           class_confidence == other.class_confidence &&
           object_to_body == other.object_to_body &&
           body_to_world == other.body_to_world &&
           velocity == other.velocity &&
           dimensions == other.dimensions &&
           properties == other.properties;
    // clang-format on
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
