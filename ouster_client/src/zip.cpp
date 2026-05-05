#include "zip.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <stdexcept>

namespace ouster {
namespace sdk {
namespace core {

// NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init)
Zip::Zip(const uint8_t* data, size_t size, int flags) {
    zip_error_init(&error_);
    source_ = zip_source_buffer_create(data, size, 0, &error_);

    if (source_ == nullptr) {
        std::stringstream string_stream;
        string_stream << "Error reading zip blob: "
                      << zip_error_strerror(&error_);
        zip_error_fini(&error_);

        throw std::invalid_argument(string_stream.str());
    }

    archive_ = zip_open_from_source(source_, flags, &error_);
    if (archive_ == nullptr) {
        std::stringstream string_stream;
        string_stream << "Error reading zip source: "
                      << zip_error_strerror(&error_);
        zip_error_fini(&error_);
        zip_source_free(source_);

        throw std::invalid_argument(string_stream.str());
    }
}

Zip::Zip()
    : Zip(nullptr, 0, 0) {  // NOLINT(cppcoreguidelines-pro-type-member-init)
}

Zip::Zip(const std::vector<uint8_t>& zip_bytes)
    : Zip(zip_bytes.data(), zip_bytes.size(),
          ZIP_RDONLY) {  // NOLINT(cppcoreguidelines-pro-type-member-init)
}

Zip::~Zip() {
    if (archive_ != nullptr) {
        zip_close(archive_);
    } else if (source_ != nullptr) {
        zip_source_free(source_);
    }
    zip_error_fini(&error_);
}

std::vector<uint8_t> Zip::get_file(const std::string& filename) {
    // ZIP_FL_NOCASE makes the search case-insensitive for ASCII characters
    zip_int64_t index =
        zip_name_locate(archive_, filename.c_str(), ZIP_FL_NOCASE);
    if (index < 0) {
        throw std::invalid_argument("Error reading " + filename +
                                    " from zip archive: No such file");
    }

    zip_uint64_t u_index = static_cast<zip_uint64_t>(index);

    zip_file_t* fzip = zip_fopen_index(archive_, u_index, 0);
    if (fzip == nullptr) {
        auto err_ptr = zip_get_error(archive_);
        std::stringstream string_stream;
        string_stream << "Error opening " << filename
                      << " from zip archive: " << zip_error_strerror(err_ptr);
        throw std::runtime_error(string_stream.str());
    }

    zip_stat_t info;
    zip_stat_init(&info);
    if (zip_stat_index(archive_, u_index, 0, &info) != 0) {
        auto err_ptr = zip_get_error(archive_);
        auto error_str = zip_error_strerror(err_ptr);
        zip_fclose(fzip);
        throw std::invalid_argument("Error reading " + filename +
                                    " from zip archive: " + error_str);
    }

    std::vector<uint8_t> out;

    if ((info.valid & ZIP_STAT_SIZE) != 0u) {
        out.resize(info.size);
        zip_fread(fzip, out.data(), info.size);
    } else {
        // no known size for the file, read it by 1kb chunks
        std::array<uint8_t, 1024> buffer{};
        while (true) {
            zip_int64_t bytes = zip_fread(fzip, buffer.data(), sizeof(buffer));

            if (bytes < 0) {
                // error occurred, return empty
                throw std::runtime_error("Error reading " + filename +
                                         " from zip archive: read error");
            }

            out.insert(out.end(), buffer.begin(), buffer.begin() + bytes);

            if (static_cast<size_t>(bytes) < sizeof(buffer)) {
                // read the last chunk, get out of the loop
                break;
            }
        }
    }

    zip_fclose(fzip);

    if (out.empty()) {
        throw std::runtime_error("Zip contained " + filename +
                                 ", but it is empty");
    }
    return out;
}
std::string Zip::get_file_as_string(const std::string& filename) {
    auto blob = get_file(filename);
    if (blob.empty()) {
        return {};
    }
    return std::string(blob.begin(), blob.end());
}

// TODO[tws] consider refactoring to use move semantics so that we don't
// have to copy blobs
void Zip::add_file(const std::string& name, const std::vector<uint8_t>& data) {
    // make a copy of the data since zip_source_buffer takes ownership
    size_t size = data.size();
    if (size == 0) {
        throw std::runtime_error("Cannot add empty file to zip archive: " +
                                 name);
    }
    void* data_copy = malloc(size);  // NOLINT(cppcoreguidelines-no-malloc,
                                     // cppcoreguidelines-owning-memory)
    if (data_copy == nullptr) {
        zip_discard(archive_);
        throw std::runtime_error("Error allocating memory for zip file");
    }
    memcpy(data_copy, data.data(), size);
    zip_source_t* src = zip_source_buffer(archive_, data_copy, size,
                                          1);  // Frees data when done

    if (src == nullptr) {
        zip_error_t* error = zip_get_error(archive_);

        std::stringstream string_stream;
        string_stream << "Error creating zip source for \"" << name
                      << "\": " << zip_error_strerror(error);

        zip_source_free(src);
        zip_discard(archive_);

        throw std::runtime_error(string_stream.str());
    }

    if (zip_file_add(archive_, name.c_str(), src, ZIP_FL_ENC_GUESS) < 0) {
        zip_error_t* error = zip_get_error(archive_);

        std::stringstream string_stream;
        string_stream << "Error adding \"" << name << "\" zip file to archive: "
                      << zip_error_strerror(error);

        zip_source_free(src);
        zip_discard(archive_);

        throw std::runtime_error(string_stream.str());
    }
}

void Zip::add_file(const std::string& name, const std::string& data) {
    add_file(name, std::vector<uint8_t>(data.begin(), data.end()));
}

std::vector<uint8_t> Zip::to_bytes() {
    // important - the actual writing of zip archive is done by zip_close()
    // which will free the source unless we keep it alive like this
    zip_source_keep(source_);
    zip_close(archive_);
    archive_ = nullptr;  // prevent double close in destructor

    std::vector<uint8_t> out;

    zip_source_open(source_);
    zip_source_seek(source_, 0, SEEK_END);
    out.resize(zip_source_tell(source_));

    zip_source_seek(source_, 0, SEEK_SET);
    zip_source_read(source_, out.data(), out.size());

    zip_source_close(source_);
    zip_source_free(source_);
    source_ = nullptr;  // prevent double free in destructor

    return out;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
