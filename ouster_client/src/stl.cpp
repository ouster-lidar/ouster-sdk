#include "ouster/stl.h"

#include <cstdint>
#include <fstream>
#include <vector>

#include "ouster/sha256.h"
#include "ouster/vector_streambuf.h"

namespace ouster {
namespace sdk {
namespace core {

namespace {
std::string get_basename(const std::string& path) {
    size_t last_slash_pos = path.find_last_of("/\\");
    if (last_slash_pos == std::string::npos) {
        return path;
    }
    return path.substr(last_slash_pos + 1);
}
}  // namespace

void Stl::load_from_stream(std::istream& stream) {
    VectorStreamBuf sbuf(&blob_);
    std::ostream out(&sbuf);
    out << stream.rdbuf();
    if (!stream) {
        throw std::runtime_error("Error reading STL from stream");
    }
}

Stl::Stl(const std::string& file_path) : filename(get_basename(file_path)) {
    VectorStreamBuf sbuf(&blob_);
    std::ostream out(&sbuf);
    std::ifstream file(file_path, std::ios::in | std::ios::binary);
    if (!file) {
        throw std::runtime_error("Error opening " + file_path);
    }
    load_from_stream(file);
}

Stl::Stl(std::istream& file) { load_from_stream(file); }

Stl::Stl(std::vector<uint8_t> blob) : blob_(std::move(blob)) {}

std::vector<uint8_t> Stl::blob() const { return blob_; }

Sha256 Stl::hash() const {
    return Sha256::hash_blobs({{blob().data(), blob().size()}});
};

Mesh Stl::to_mesh() const {
    Mesh mesh;
    if (!mesh.load_from_stl_bytes(blob_)) {
        throw std::runtime_error("Unable to parse STL");
    }
    return mesh;
}

bool operator==(const Stl& lhs, const Stl& rhs) {
    return lhs.blob_ == rhs.blob_ &&
           lhs.coordinate_frame == rhs.coordinate_frame;
}

bool operator!=(const Stl& lhs, const Stl& rhs) { return !(lhs == rhs); }

bool Stl::string_to_coordinate_frame(const std::string& str,
                                     Stl::CoordinateFrame& coordinate_frame) {
    // NOTE - no "NONE" because that is the default value and should not be set
    if (str == "BODY") {
        coordinate_frame = Stl::CoordinateFrame::BODY;
    } else if (str == "SENSOR") {
        coordinate_frame = Stl::CoordinateFrame::SENSOR;
    } else {
        return false;
    }
    return true;
}

std::string to_string(Stl::CoordinateFrame coordinate_frame) {
    switch (coordinate_frame) {
        case Stl::CoordinateFrame::NONE:
            return "NONE";
        case Stl::CoordinateFrame::BODY:
            return "BODY";
        case Stl::CoordinateFrame::SENSOR:
            return "SENSOR";
        default:
            break;
    };
    return "UNKNOWN";
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
