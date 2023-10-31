/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <json/json.h>

/* Helper function which produces a combination of the two roots, with
 * any value in root_orig that exists in root_new replaced with the root_new
 * value. The vector changed indicates each field changed in the result
 * from the original
 */

namespace ouster {

Json::Value combined(const Json::Value& root_orig, const Json::Value& root_new,
                     std::vector<std::string>& changed);

}
