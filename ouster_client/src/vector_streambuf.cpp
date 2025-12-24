#include "ouster/vector_streambuf.h"

#include <cstddef>
#include <cstdint>
#include <streambuf>

namespace ouster {
namespace sdk {
namespace core {

VectorStreamBuf::VectorStreamBuf(const ByteVector* buffer)
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    : buffer_(const_cast<ByteVector*>(
          buffer)) {  // std::streambuf's setg and setp methods require
                      // non-const pointer args.
    if (buffer_ == nullptr) {
        throw std::invalid_argument("VectorStreamBuf: buffer pointer is null");
    }
    setg(reinterpret_cast<char*>(buffer_->data()),
         reinterpret_cast<char*>(buffer_->data()),
         reinterpret_cast<char*>(buffer_->data()) + buffer_->size());
    setp(reinterpret_cast<char*>(buffer_->data()),
         reinterpret_cast<char*>(buffer_->data()) + buffer_->size());
}

VectorStreamBuf::int_type VectorStreamBuf::overflow(int_type character) {
    // TODO[tws] consider writing at the current position instead of
    // appending. Override the overflow function to write one character at a
    // time.
    if (character != EOF) {
        buffer_->push_back(static_cast<uint8_t>(character));
    }
    return character;
}

std::streamsize VectorStreamBuf::xsputn(const char* source,
                                        std::streamsize count) {
    // TODO[tws] consider writing at the current position instead of
    // appending.
    buffer_->insert(buffer_->end(), reinterpret_cast<const uint8_t*>(source),
                    reinterpret_cast<const uint8_t*>(source) + count);
    return count;
}

std::streampos VectorStreamBuf::seekoff(std::streamoff off,
                                        std::ios_base::seekdir way,
                                        std::ios_base::openmode openmode) {
    // TODO[tws] consider updating this method to support write mode.
    // Support seeking in input (read-only) mode
    if ((openmode & std::ios_base::in) == 0) {
        return -1;
    }

    ptrdiff_t new_pos = 0;
    if (way == std::ios_base::beg) {
        new_pos = off;
    } else if (way == std::ios_base::cur) {
        new_pos = gptr() - eback() + off;
    } else if (way == std::ios_base::end) {
        new_pos = static_cast<ptrdiff_t>(buffer_->size()) + off;
    } else {
        return -1;
    }

    if (new_pos < 0 || static_cast<size_t>(new_pos) > buffer_->size()) {
        return -1;
    }

    setg(eback(), eback() + new_pos, eback() + buffer_->size());
    return new_pos;
}

std::streampos VectorStreamBuf::seekpos(std::streampos position,
                                        std::ios_base::openmode openmode) {
    return seekoff(position, std::ios_base::beg, openmode);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
