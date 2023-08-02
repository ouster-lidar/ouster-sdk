/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/util.h"

#include <iostream>

namespace ouster {

void combined_helper(const Json::Value& root_orig, const Json::Value& root_new,
                     Json::Value& result, std::string changed_str,
                     std::vector<std::string>& changed) {
    // Iterate over new root - any
    for (Json::Value::const_iterator itr = root_new.begin();
         itr != root_new.end(); itr++) {
        std::string key = itr.key().asString();
        std::string new_changed_str =
            (changed_str == "") ? key : changed_str + "." + key;
        if (root_orig.isMember(key) && root_new[key].isObject() &&
            root_orig[key].isObject()) {
            // need to crawl to collect changed_str output
            combined_helper(root_orig[key], root_new[key], result[key],
                            new_changed_str, changed);
        } else {
            // CASE: root_orig does not have key OR
            // CASE: key is leaf OR
            // CASE: key is not matched as leaf/object in root_new vs root_orig

            bool change = false;

            // Unfortunately we have to handle the Ints and Doubles because 0 is
            // different from 0.0 in output
            // TODO - find differnt way so we don't have to do this
            if (!root_orig.isMember(key) || root_new[key].isObject()) {
                change = true;
            } else if ((root_new[key].isString() || root_new[key].isBool()) &&
                       (root_orig[key] != root_new[key])) {
                change = true;
            } else if (root_new[key].isIntegral() &&
                       (root_orig[key].asInt() != root_new[key].asInt())) {
                change = true;
            } else if (root_new[key].isDouble() && (root_orig[key].asDouble() !=
                                                    root_new[key].asDouble())) {
                change = true;
            } else if (root_new[key].isArray()) {
                // NOTE: this does not handle the array of arrays that DF
                // currently has
                for (size_t i = 0; i < root_new[key].size(); i++) {
                    if (root_new[key][(int)i].asDouble() !=
                        root_orig[key][(int)i].asDouble()) {
                        change = true;
                        continue;
                    }
                }
            }

            if (change) {
                result[key] = root_new[key];
                changed.push_back(new_changed_str);
            }
        }
    }
}  // namespace ouster

Json::Value combined(const Json::Value& root_orig, const Json::Value& root_new,
                     std::vector<std::string>& changed) {
    Json::Value result{};

    // make sure changed is empty to start
    changed.clear();

    // set result equal to root_orig to capture all keys in root_orig
    result = root_orig;

    combined_helper(root_orig, root_new, result, "", changed);
    return result;
}

}  // namespace ouster
