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

#include "ouster/core/camera_frame.h"
#include "ouster/core/field.h"
#include "ouster/core/frame.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/object.h"

namespace ouster {
namespace sdk {
namespace core {

FrameSet::FrameSet()
    : frames_(),
      fields_(std::make_shared<std::unordered_map<std::string, Field>>()),
      objects_(std::make_shared<std::unordered_map<std::string, std::vector<Object>>>()) {}

FrameSet::FrameSet(std::vector<std::shared_ptr<Frame>> frames) : FrameSet() {
    // Clang-tidy stupidly doesn't like this, but moving frames_ initialization into the member
    // initializer list is a compile error
    frames_ = std::move(frames);  // NOLINT(cppcoreguidelines-prefer-member-initializer)
}

FrameSet::FrameSet(const std::vector<std::shared_ptr<LidarFrame>>& frames) : FrameSet() {
    frames_.reserve(frames.size());
    for (const auto& frame : frames) {
        frames_.push_back(frame);
    }
}

FrameSet::FrameSet(std::initializer_list<std::shared_ptr<LidarFrame>> frames) : FrameSet() {
    frames_.reserve(frames.size());
    for (const auto& frame : frames) {
        frames_.push_back(frame);
    }
}

std::vector<std::shared_ptr<Frame>>::iterator FrameSet::begin() {
    return frames_.begin();
}

std::vector<std::shared_ptr<Frame>>::iterator FrameSet::end() {
    return frames_.end();
}

std::vector<std::shared_ptr<Frame>>::const_iterator FrameSet::begin() const {
    return frames_.cbegin();
}

std::vector<std::shared_ptr<Frame>>::const_iterator FrameSet::end() const {
    return frames_.cend();
}

const std::shared_ptr<Frame>& FrameSet::operator[](size_t index) const {
    try {
        return frames_.at(index);
    } catch (std::out_of_range& e) {
        throw std::out_of_range("Requested frame out of range");
    }
}

std::shared_ptr<Frame>& FrameSet::operator[](size_t index) {
    try {
        return frames_.at(index);
    } catch (std::out_of_range& e) {
        throw std::out_of_range("Requested frame out of range");
    }
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

Field& FrameSet::add_field(const std::string& name, const FieldDescriptor& desc, bool zero) {
    if (has_field(name)) {
        throw std::invalid_argument("Duplicated field '" + name + "'");
    }

    fields()[name] = Field{desc, FieldClass::COLLATION_FIELD, zero};

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

std::vector<std::shared_ptr<Frame>>& FrameSet::frames() {
    return frames_;
}

const std::vector<std::shared_ptr<Frame>>& FrameSet::frames() const {
    return frames_;
}

std::vector<std::shared_ptr<LidarFrame>> FrameSet::valid_lidar_frames() const {
    return valid_frames_of<LidarFrame>();
}

std::vector<std::shared_ptr<CameraFrame>> FrameSet::valid_camera_frames() const {
    return valid_frames_of<CameraFrame>();
}

void FrameSet::swap(FrameSet& other) noexcept {
    std::swap(frames(), other.frames());
    std::swap(fields(), other.fields());
    std::swap(objects(), other.objects());
}

FrameSet FrameSet::clone() const {
    FrameSet out{};

    out.frames_.reserve(frames_.size());
    for (const auto& ptr : frames_) {
        if (ptr) {
            out.frames_.push_back(ptr->clone());
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
        const bool both_empty = !a[i] && !b[i];
        const bool both_present = a[i] && b[i];
        if (both_empty) {
            continue;
        }
        // Frame::equals() is virtual, so this compares the frames by their
        // concrete types (LidarFrame, CameraFrame, ...) without downcasting
        // here, and reports frames of differing types as unequal.
        if (!both_present || !a[i]->equals(*b[i])) {
            return false;
        }
    }

    // frames are equal, return true if fields and object lists are equal too
    return a.fields() == b.fields() && a.objects() == b.objects();
}

bool operator!=(const FrameSet& a, const FrameSet& b) {
    return !(a == b);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster

void std::swap(ouster::sdk::core::FrameSet& a, ouster::sdk::core::FrameSet& b) {
    a.swap(b);
}
