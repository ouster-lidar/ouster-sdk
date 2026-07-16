#include "ouster/core/frame_set_source.h"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {

std::set<std::string> FrameSetSourceMetadataSet::keys() const {
    std::set<std::string> keys_set;
    for (const auto& pair : entries) {
        keys_set.insert(pair.first);
    }
    return keys_set;
}

FrameSet FrameSetSource::operator[](int index) const {
    auto iter = begin();
    if (index < 0) {
        auto len = end() - begin();
        iter += len + index;
    } else {
        iter += index;
    }
    if (iter == end()) {
        throw std::out_of_range("Indexed past the end of the frame set source.");
    }
    return *iter;
}

size_t FrameSetSource::size() const {
    return end() - begin();
}

FrameSetIterator FrameSetSource::end() const {
    return FrameSetIterator(this);
}

bool FrameSetSource::is_live() const {
    return false;
}

bool FrameSetSource::is_indexed() const {
    return false;
}

bool FrameSetSource::is_collated() const {
    return false;
}

bool FrameSetSource::contains_collations() const {
    return false;
}

const std::vector<std::pair<uint64_t, uint64_t>>& FrameSetSource::full_index() const {
    throw std::runtime_error("'full_index' not supported on unindexed frame set sources.");
}

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>& FrameSetSource::individual_index()
    const {
    throw std::runtime_error("'individual_index' not supported on unindexed frame set sources.");
}

const std::vector<size_t>& FrameSetSource::frames_num() const {
    throw std::runtime_error("'frames_num' not supported on unindexed frame set sources.");
}

const std::vector<size_t>& FrameSetSource::scans_num() const {
    return frames_num();
}

// TODO[tws] law of demeter
void FrameSetSource::add_metadata(const std::string& name, FrameSetSourceMetadata obj) {
    metadata_.entries.emplace(name, std::move(obj));
}

// TODO[tws] law of demeter
bool FrameSetSource::has_metadata(const std::string& key) const {
    return metadata_.entries.find(key) != metadata_.entries.end();
}

// TODO[tws] law of demeter
std::set<std::string> FrameSetSource::metadata_keys() const {
    return metadata_.keys();
}

const FrameSetSourceMetadata& FrameSetSource::metadata(const std::string& name) const {
    auto it = metadata_.entries.find(name);
    if (it != metadata_.entries.end()) {
        return it->second;
    } else {
        throw std::out_of_range("Metadata with name '" + name + "' not found.");
    }
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
