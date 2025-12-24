#include "ouster/osf/memory_mapped_osf_file.h"

#include <cstdint>
#include <sstream>
#include <string>
#include <utility>

#include "ouster/osf/buffer.h"
#include "ouster/osf/file.h"
#include "ouster/osf/impl/compat_ops.h"
#include "ouster/osf/offset.h"

namespace ouster {
namespace sdk {
namespace osf {

MemoryMappedOsfFile::MemoryMappedOsfFile(const std::string& filename)
    : OsfFile(filename), size_(file_size(filename)) {
    file_buf_ = mmap_open(filename, memmap_handle_);
    initialize_header_and_metadata();
}

MemoryMappedOsfFile::~MemoryMappedOsfFile() { close(); }

void MemoryMappedOsfFile::close() {
    state_ = FileState::BAD;
    if (file_buf_ != nullptr) {
        if (!mmap_close(file_buf_, size_, memmap_handle_)) {
            error();
            return;
        }
        file_buf_ = nullptr;
    }
}

uint8_t* MemoryMappedOsfFile::buf_raw_ptr(OsfOffset& base_offset,
                                          OsfOffset& offset) {
    if (!good()) {
        throw std::logic_error("bad osf file");
    }
    if (base_offset.offset() + offset.offset() >= size_) {
        throw std::runtime_error("EOF reached");
    }
    if (base_offset.size() > 0 && offset.size() > base_offset.size()) {
        std::stringstream stream;
        stream << "error: requested read is larger than the base offsets size: "
               << base_offset.size() << " requested: " << offset.size();
        throw std::runtime_error(stream.str());
    }

    return file_buf_ + base_offset.offset() + offset.offset();
}

OsfBuffer MemoryMappedOsfFile::read(OsfOffset offset) {
    OsfOffset no_offset = {0, 0};
    return read(no_offset, offset);
}

OsfBuffer MemoryMappedOsfFile::read(OsfOffset base_offset, OsfOffset offset) {
    OsfBuffer buffer;
    buffer.load_data(buf_raw_ptr(base_offset, offset), offset.size());

    return buffer;
}

MemoryMappedOsfFile::MemoryMappedOsfFile(MemoryMappedOsfFile&& other) noexcept
    : OsfFile(std::move(other)) {
    other.file_buf_ = nullptr;
    other.state_ = FileState::BAD;
}

MemoryMappedOsfFile& MemoryMappedOsfFile::operator=(
    MemoryMappedOsfFile&& other) noexcept {
    OsfFile::operator=(std::move(other));
    if (this != &other) {
        close();
        file_buf_ = other.file_buf_;
        other.file_buf_ = nullptr;
    }
    return *this;
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
