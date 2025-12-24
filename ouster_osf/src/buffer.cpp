#include "ouster/osf/buffer.h"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ouster {
namespace sdk {
namespace osf {

OsfBuffer::~OsfBuffer() { reset(); }

void OsfBuffer::reset() {
    memory_mapped_data_ = nullptr;
    size_ = 0;
    offset_ = 0;
    non_memory_mapped_data_.reset();
    has_value_ = false;
}

OsfBuffer::OsfBuffer()
    : memory_mapped_data_(nullptr), size_(0), offset_(0), has_value_(false) {}

OsfBuffer::OsfBuffer(const OsfBuffer& other)
    : memory_mapped_data_(other.memory_mapped_data_),
      non_memory_mapped_data_(other.non_memory_mapped_data_),
      size_(other.size()),
      offset_(other.offset_),
      has_value_(other.has_value_) {}

OsfBuffer& OsfBuffer::operator=(const OsfBuffer& other) {
    if (this != &other) {
        memory_mapped_data_ = other.memory_mapped_data_;
        non_memory_mapped_data_ = other.non_memory_mapped_data_;
        size_ = other.size();
        offset_ = other.offset_;
        has_value_ = other.has_value_;
    }
    return *this;
}

bool OsfBuffer::operator==(const OsfBuffer& other) const {
    if (this == &other) {
        return true;
    }
    if (size_ != other.size_) {
        return false;
    }
    if (offset_ != other.offset_) {
        return false;
    }
    if (memory_mapped_data_ != other.memory_mapped_data_) {
        return false;
    }
    if (non_memory_mapped_data_ != other.non_memory_mapped_data_) {
        return false;
    }
    return true;
}

void OsfBuffer::load_data(const uint8_t* data, uint64_t size) {
    if (data == nullptr) {
        throw std::logic_error("Invalid data");
    }
    memory_mapped_data_ = data;
    size_ = size;
    offset_ = 0;
    has_value_ = true;
}

void OsfBuffer::load_data(std::vector<uint8_t>&& data) {
    if (data.empty()) {
        throw std::logic_error("Invalid data");
    }
    non_memory_mapped_data_ =
        std::make_shared<std::vector<uint8_t>>(std::move(data));
    size_ = non_memory_mapped_data_->size();
    offset_ = 0;
    has_value_ = true;
}

void OsfBuffer::load_data(const std::vector<uint8_t>& data) {
    if (data.empty()) {
        throw std::logic_error("Invalid data");
    }
    non_memory_mapped_data_ = std::make_shared<std::vector<uint8_t>>(data);
    size_ = non_memory_mapped_data_->size();
    offset_ = 0;
    has_value_ = true;
}

void OsfBuffer::load_data(const class OsfBuffer& base_buffer, uint64_t offset,
                          uint64_t size) {
    if (!base_buffer.has_value()) {
        throw std::logic_error("Invalid base buffer");
    }

    if (offset + size > base_buffer.size_) {
        throw std::logic_error("Invalid offset/size for base buffer");
    }
    offset_ = base_buffer.offset_ + offset;
    size_ = size;

    if (base_buffer.non_memory_mapped_data_ != nullptr) {
        non_memory_mapped_data_ = base_buffer.non_memory_mapped_data_;
        has_value_ = true;
    } else if (base_buffer.memory_mapped_data_ != nullptr) {
        memory_mapped_data_ = base_buffer.memory_mapped_data_;
        has_value_ = true;
    } else {
        throw std::logic_error("Invalid base buffer data");
    }
}

const uint8_t* OsfBuffer::data() const {
    if (!has_value_) {
        // TODO[tws] should be a logic_error, or even better, OsfBuffer should
        // never be constructed uninitialized
        return nullptr;
    }

    if (memory_mapped_data_ != nullptr) {
        return memory_mapped_data_ + offset_;
    } else {
        return non_memory_mapped_data_->data() + offset_;
    }
}

const uint8_t* OsfBuffer::cbegin() const { return data(); }

const uint8_t* OsfBuffer::cend() const { return data() + size(); }

uint64_t OsfBuffer::size() const { return size_; }

bool OsfBuffer::has_value() const { return has_value_; }

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
