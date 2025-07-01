#include "ouster/scan_source.h"

namespace ouster {
namespace core {
std::vector<std::shared_ptr<LidarScan>> ScanSource::operator[](
    int start) const {
    auto iter = begin();
    if (start < 0) {
        auto len = end() - begin();
        iter += len + start;
    } else {
        iter += start;
    }
    if (iter == end()) {
        throw std::out_of_range("Indexed past the end of the scan source.");
    }
    return {*iter};
}

size_t ScanSource::size() const { return end() - begin(); }

ScanIterator ScanSource::end() const { return ScanIterator(this); }

bool ScanSource::is_live() const { return false; }

bool ScanSource::is_indexed() const { return false; }

const std::vector<std::pair<uint64_t, uint64_t>>& ScanSource::full_index()
    const {
    throw std::runtime_error(
        "'full_index' not supported on unindexed scan sources.");
}

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
ScanSource::individual_index() const {
    throw std::runtime_error(
        "'individual_index' not supported on unindexed scan sources.");
}

const std::vector<size_t>& ScanSource::scans_num() const {
    throw std::runtime_error(
        "'scans_num' not supported on unindexed scan sources.");
}

}  // namespace core
}  // namespace ouster
