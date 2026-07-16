/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/frame_set.h"

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

FrameSet::FrameSet()
    : frames_(),
      fields_(std::make_shared<std::unordered_map<std::string, Field>>()),
      objects_(std::make_shared<std::unordered_map<std::string, std::vector<Object>>>()) {}

FrameSet::FrameSet(const std::vector<std::shared_ptr<LidarFrame>>& frames)
    : frames_(frames),
      fields_(std::make_shared<std::unordered_map<std::string, Field>>()),
      objects_(std::make_shared<std::unordered_map<std::string, std::vector<Object>>>()) {}

FrameSet::FrameSet(std::vector<std::shared_ptr<LidarFrame>>&& frames) : FrameSet() {
    frames_.swap(frames);
}

FrameSet::FrameSet(std::initializer_list<std::shared_ptr<LidarFrame>> frames)
    : frames_(frames),
      fields_(std::make_shared<std::unordered_map<std::string, Field>>()),
      objects_(std::make_shared<std::unordered_map<std::string, std::vector<Object>>>()) {}

std::vector<std::shared_ptr<LidarFrame>>::iterator FrameSet::begin() {
    return frames_.begin();
}

std::vector<std::shared_ptr<LidarFrame>>::iterator FrameSet::end() {
    return frames_.end();
}

std::vector<std::shared_ptr<LidarFrame>>::const_iterator FrameSet::begin() const {
    return frames_.cbegin();
}

std::vector<std::shared_ptr<LidarFrame>>::const_iterator FrameSet::end() const {
    return frames_.cend();
}

const std::shared_ptr<LidarFrame>& FrameSet::operator[](size_t index) const {
    try {
        return frames_.at(index);
    } catch (std::out_of_range& e) {
        throw std::out_of_range("Requested frame out of range");
    }
}

std::shared_ptr<LidarFrame>& FrameSet::operator[](size_t index) {
    return const_cast<std::shared_ptr<LidarFrame>&>((*const_cast<const FrameSet*>(this))[index]);
}

size_t FrameSet::size() const {
    return frames_.size();
}

/**
 * Some of these are direct copy of LidarFrame field accessors and some others
 * are ever so slightly different.
 * In an ideal world we would make a base class for these, but the differences
 * make it not worth the refactoring trouble at the moment.
 */

Field& FrameSet::add_field(const std::string& name, const FieldDescriptor& desc) {
    if (has_field(name)) {
        throw std::invalid_argument("Duplicated field '" + name + "'");
    }

    fields()[name] = Field{desc, FieldClass::COLLATION_FIELD};

    return fields()[name];
}

Field FrameSet::del_field(const std::string& name) {
    if (!has_field(name)) {
        throw std::invalid_argument("Attempted deleting non existing field '" + name + "'");
    }

    Field ptr;
    field(name).swap(ptr);
    fields().erase(name);
    return ptr;
}

bool FrameSet::has_field(const std::string& name) const {
    return fields().count(name) > 0;
}

const Field& FrameSet::field(const std::string& name) const {
    try {
        return fields().at(name);
    } catch (std::out_of_range& e) {
        throw std::out_of_range("Field '" + name + "' not found in FrameSet.");
    }
}

Field& FrameSet::field(const std::string& name) {
    return const_cast<Field&>(const_cast<const FrameSet*>(this)->field(name));
}

std::unordered_map<std::string, Field>& FrameSet::fields() {
    return *fields_;
}

const std::unordered_map<std::string, Field>& FrameSet::fields() const {
    return *fields_;
}

std::unordered_map<std::string, std::vector<Object>>& FrameSet::objects() {
    return *objects_;
}

const std::unordered_map<std::string, std::vector<Object>>& FrameSet::objects() const {
    return *objects_;
}

std::vector<std::shared_ptr<LidarFrame>>& FrameSet::frames() {
    return frames_;
}

const std::vector<std::shared_ptr<LidarFrame>>& FrameSet::frames() const {
    return frames_;
}

OUSTER_DIAGNOSTIC_PUSH
OUSTER_DIAGNOSTIC_IGNORE_DEPRECATED
std::vector<std::shared_ptr<LidarFrame>>& FrameSet::scans() {
    return frames();
}
const std::vector<std::shared_ptr<LidarFrame>>& FrameSet::scans() const {
    return frames();
}
OUSTER_DIAGNOSTIC_POP

void FrameSet::swap(FrameSet& other) noexcept {
    std::swap(frames(), other.frames());
    std::swap(fields(), other.fields());
    std::swap(objects(), other.objects());
}

FrameSet FrameSet::clone() const {
    FrameSet out{};

    for (const auto& ptr : frames_) {
        if (ptr) {
            out.frames_.push_back(std::make_shared<LidarFrame>(*ptr));
        } else {
            out.frames_.push_back({});
        }
    }
    out.fields() = fields();
    out.objects() = objects();
    return out;
}

bool operator==(const FrameSet& a, const FrameSet& b) {
    if (a.size() != b.size()) {
        return false;
    }

    for (size_t i = 0; i < a.size(); ++i) {
        bool both_empty = !a[i] && !b[i];
        bool frames_equal = a[i] && b[i] && (*a[i] == *b[i]);
        // return early if frames aren't equal
        if (!(both_empty || frames_equal)) {
            return false;
        }
    }

    // frames are equal, return true if fields and object lists are equal too
    return a.fields() == b.fields() && a.objects() == b.objects();
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster

void std::swap(ouster::sdk::core::FrameSet& a, ouster::sdk::core::FrameSet& b) {
    a.swap(b);
}
