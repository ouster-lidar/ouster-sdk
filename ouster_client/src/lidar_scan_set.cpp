/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/lidar_scan_set.h"

#include <cstddef>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {

LidarScanSet::LidarScanSet()
    : scans_(),
      fields_(std::make_shared<std::unordered_map<std::string, Field>>()) {}

LidarScanSet::LidarScanSet(const std::vector<std::shared_ptr<LidarScan>>& scans)
    : scans_(scans),
      fields_(std::make_shared<std::unordered_map<std::string, Field>>()) {}

LidarScanSet::LidarScanSet(std::vector<std::shared_ptr<LidarScan>>&& scans)
    : LidarScanSet() {
    scans_.swap(scans);
}

LidarScanSet::LidarScanSet(
    std::initializer_list<std::shared_ptr<LidarScan>> scans)
    : scans_(scans),
      fields_(std::make_shared<std::unordered_map<std::string, Field>>()) {}

std::vector<std::shared_ptr<LidarScan>>::iterator LidarScanSet::begin() {
    return scans_.begin();
}

std::vector<std::shared_ptr<LidarScan>>::iterator LidarScanSet::end() {
    return scans_.end();
}

std::vector<std::shared_ptr<LidarScan>>::const_iterator LidarScanSet::begin()
    const {
    return scans_.cbegin();
}

std::vector<std::shared_ptr<LidarScan>>::const_iterator LidarScanSet::end()
    const {
    return scans_.cend();
}

const std::shared_ptr<LidarScan>& LidarScanSet::operator[](int index) const {
    try {
        return scans_.at(index);
    } catch (std::out_of_range& e) {
        throw std::out_of_range("Requested scan out of range");
    }
}

std::shared_ptr<LidarScan>& LidarScanSet::operator[](int index) {
    return const_cast<std::shared_ptr<LidarScan>&>(
        (*const_cast<const LidarScanSet*>(this))[index]);
}

size_t LidarScanSet::size() const { return scans_.size(); }

/**
 * Some of these are direct copy of LidarScan field accessors and some others
 * are ever so slightly different.
 * In an ideal world we would make a base class for these, but the differences
 * make it not worth the refactoring trouble at the moment.
 */

Field& LidarScanSet::add_field(const std::string& name, FieldDescriptor desc) {
    if (has_field(name)) {
        throw std::invalid_argument("Duplicated field '" + name + "'");
    }

    fields()[name] = Field{desc, FieldClass::COLLATION_FIELD};

    return fields()[name];
}

Field LidarScanSet::del_field(const std::string& name) {
    if (!has_field(name)) {
        throw std::invalid_argument("Attempted deleting non existing field '" +
                                    name + "'");
    }

    Field ptr;
    field(name).swap(ptr);
    fields().erase(name);
    return ptr;
}

bool LidarScanSet::has_field(const std::string& name) const {
    return fields().count(name) > 0;
}

const Field& LidarScanSet::field(const std::string& name) const {
    try {
        return fields().at(name);
    } catch (std::out_of_range& e) {
        throw std::out_of_range("Field '" + name +
                                "' not found in LidarScanSet.");
    }
}

Field& LidarScanSet::field(const std::string& name) {
    return const_cast<Field&>(
        const_cast<const LidarScanSet*>(this)->field(name));
}

std::unordered_map<std::string, Field>& LidarScanSet::fields() {
    return *fields_;
}

const std::unordered_map<std::string, Field>& LidarScanSet::fields() const {
    return *fields_;
}

std::vector<std::shared_ptr<LidarScan>>& LidarScanSet::scans() {
    return scans_;
}

const std::vector<std::shared_ptr<LidarScan>>& LidarScanSet::scans() const {
    return scans_;
}

void LidarScanSet::swap(LidarScanSet& other) noexcept {
    std::swap(scans_, other.scans_);
    std::swap(fields_, other.fields_);
}

LidarScanSet LidarScanSet::clone() const {
    LidarScanSet out{};

    for (const auto& ptr : scans_) {
        if (ptr) {
            out.scans_.push_back(std::make_shared<LidarScan>(*ptr));
        } else {
            out.scans_.push_back({});
        }
    }
    out.fields() = fields();
    return out;
}

bool operator==(const LidarScanSet& a, const LidarScanSet& b) {
    if (a.size() != b.size()) {
        return false;
    }

    for (size_t i = 0; i < a.size(); ++i) {
        bool both_empty = !a[i] && !b[i];
        bool scans_equal = a[i] && b[i] && (*a[i] == *b[i]);
        // return early if scans aren't equal
        if (!(both_empty || scans_equal)) {
            return false;
        }
    }

    // scans are equal, return true if fields are equal too
    return a.fields() == b.fields();
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster

void std::swap(ouster::sdk::core::LidarScanSet& a,
               ouster::sdk::core::LidarScanSet& b) {
    a.swap(b);
}
