#include "ouster/osf/callback_osf_file.h"

namespace ouster {
namespace sdk {
namespace osf {

CallbackOsfFile::CallbackOsfFile(CallbackOsfFile::fetch_callback_t callback, uint64_t file_size,
                                 const std::string& name_or_url)
    : OsfFile(name_or_url), file_size_(file_size), fetch_callback_(std::move(callback)) {
    initialize_header_and_metadata();
}

CallbackOsfFile::~CallbackOsfFile() {}

CallbackOsfFile::CallbackOsfFile(CallbackOsfFile&& other)
    : OsfFile(std::move(other)),
      file_size_(other.file_size_),
      fetch_callback_(std::move(other.fetch_callback_)) {}

CallbackOsfFile& CallbackOsfFile::operator=(CallbackOsfFile&& other) {
    if (this != &other) {
        OsfFile::operator=(std::move(other));
        header_chunk_ = other.header_chunk_;
        metadata_chunk_ = other.metadata_chunk_;
        file_size_ = other.file_size_;
        fetch_callback_ = std::move(other.fetch_callback_);
    }
    return *this;
}

OsfBuffer CallbackOsfFile::read(OsfOffset offset) {
    OsfOffset no_offset = {0, 0};
    return read(no_offset, offset);
}

OsfBuffer CallbackOsfFile::read(OsfOffset base_offset, OsfOffset offset) {
    if (base_offset.offset() + offset.offset() >= file_size_) {
        throw std::out_of_range("out of range osf file access");
    }
    if (base_offset.size() > 0 && offset.size() > base_offset.size()) {
        std::stringstream stream;
        stream << "error: requested read is larger than the base offsets size: "
               << base_offset.size() << " requested: " << offset.size();
        throw std::out_of_range(stream.str());
    }

    OsfOffset fetch_offset = {base_offset.offset() + offset.offset(), offset.size()};
    return fetch_callback_(fetch_offset);
}

uint64_t CallbackOsfFile::size() const {
    return file_size_;
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
