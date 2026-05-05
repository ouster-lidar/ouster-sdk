#include "ouster/osf/stream_osf_file.h"

#include <cstdint>
#include <ios>
#include <string>
#include <utility>
#include <vector>

#include "ouster/osf/buffer.h"
#include "ouster/osf/crc32.h"
#include "ouster/osf/impl/compat_ops.h"
#include "ouster/osf/impl/fb_utils.h"
#include "ouster/osf/offset.h"

namespace ouster {
namespace sdk {
namespace osf {

// ========= OsfFileFull ============

StreamOsfFile::StreamOsfFile(const std::string& filename) : OsfFile(filename) {
    file_stream_ = std::ifstream(filename, std::ios::in | std::ios::binary);
    file_stream_.seekg(0, std::ios::end);
    size_ = file_stream_.tellg();
    file_stream_.seekg(0, std::ios::beg);
    initialize_header_and_metadata();
}

StreamOsfFile::~StreamOsfFile() { close(); }

StreamOsfFile::StreamOsfFile(StreamOsfFile&& other)
    : OsfFile(std::move(other)), file_stream_(std::move(other.file_stream_)) {
    other.state_ = FileState::BAD;
}

StreamOsfFile& StreamOsfFile::operator=(StreamOsfFile&& other) {
    OsfFile::operator=(std::move(other));
    if (this != &other) {
        file_stream_ = std::move(other.file_stream_);
        other.state_ = FileState::BAD;
    }
    return *this;
};

void StreamOsfFile::close() {
    state_ = FileState::BAD;
    if (file_stream_.is_open()) {
        file_stream_.close();
        if (file_stream_.fail()) {
            error();
            return;
        }
    }
}

void StreamOsfFile::check_stream_bits() {
    if (file_stream_.eof()) {
        throw std::runtime_error("EOF reached");
    }
    if (file_stream_.fail()) {
        throw std::runtime_error("Read failure");
    }
    if (file_stream_.bad()) {
        throw std::runtime_error("IO error while reading " + path_ + ": " +
                                 get_last_error());
    }
}

StreamOsfFile& StreamOsfFile::seek(uint64_t pos) {
    if (!good()) {
        throw std::logic_error("bad osf file");
    }
    file_stream_.seekg(pos, std::ios::beg);
    check_stream_bits();
    return *this;
}

OsfBuffer StreamOsfFile::read(OsfOffset offset) {
    OsfOffset no_offset = {0, 0};
    return read(no_offset, offset);
}

OsfBuffer StreamOsfFile::read(OsfOffset base_offset, OsfOffset offset) {
    if (!good()) {
        throw std::logic_error("bad osf file");
    }

    std::vector<uint8_t> data(offset.size());
    seek(base_offset.offset() + offset.offset());

    file_stream_.read(reinterpret_cast<char*>(data.data()), offset.size());
    if (file_stream_.gcount() != static_cast<std::streamsize>(offset.size())) {
        check_stream_bits();
    }
    OsfBuffer buffer;
    buffer.load_data(std::move(data));

    return buffer;
}

uint64_t StreamOsfFile::size() const { return size_; }

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
