#include "ouster/osf/offset.h"

namespace ouster {
namespace sdk {
namespace osf {

OsfOffset::OsfOffset(const uint64_t offset_in, const uint64_t size_in)
    : offset_(offset_in), size_(size_in), valid_(true){};

bool OsfOffset::operator==(const OsfOffset& other) const {
    return offset_ == other.offset_ && size_ == other.size_;
}

uint64_t OsfOffset::offset() const { return offset_; }

uint64_t OsfOffset::size() const { return size_; }

std::ostream& operator<<(std::ostream& out_stream, const OsfOffset& offset) {
    out_stream << "OsfOffset [offset = " << offset.offset()
               << ", size = " << offset.size() << "]";
    return out_stream;
}

bool OsfOffset::valid() const { return valid_; }

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
