#include "ouster/osf/file_info.h"

#include <iostream>
#include <string>

#include "util_impl.h"

namespace ouster {
namespace OSF {

FileInfo::FileInfo(const OsfFile& osf_file) {
    if (!osf_file.valid()) {
        throw std::logic_error("provided OSF file is not valid");
    }
    auto osf_header = get_osf_header_from_buf(osf_file.buf());
    size_t session_offset = osf_header->session_offset();
    // TODO[pb]: Check fb idetifier?
    buf_ = osf_file.buf(session_offset);
    // Cache sensors map
    smap_ = read_sensors();
    // and chunk (frames) map
    cmap_ = get_map_from_file_info(*this);
}

std::string FileInfo::id() const {
    auto osf_session = get_osf_session_from_buf(buf_);
    return osf_session->id()->str();
};

bool FileInfo::isFramed() const {
    auto osf_session = get_osf_session_from_buf(buf_);
    return osf_session->session_mode() == OSF_SESSION_MODE_OSF_FRAMED;
}

OSF_SESSION_MODE FileInfo::mode() const {
    auto osf_session = get_osf_session_from_buf(buf_);
    return osf_session->session_mode();
}

OSF_FRAME_MODE FileInfo::lidar_frame_mode() const {
    auto osf_session = get_osf_session_from_buf(buf_);
    return osf_session->lidar_frame_mode();
}

uint8_t FileInfo::range_multiplier() const {
    auto osf_session = get_osf_session_from_buf(buf_);
    return osf_session->range_multiplier();
}

FileInfo::ts_t FileInfo::start_ts() const {
    auto osf_session = get_osf_session_from_buf(buf_);
    return FileInfo::ts_t(osf_session->start_ts());
}

FileInfo::ts_t FileInfo::end_ts() const {
    auto osf_session = get_osf_session_from_buf(buf_);
    return FileInfo::ts_t(osf_session->end_ts());
}

const FileInfo::sensors_map& FileInfo::sensors() const { return smap_; }

std::vector<uint64_t> FileInfo::frames_keys() const {
    std::vector<uint64_t> keys;
    keys.reserve(cmap_.size());
    for (const auto el : cmap_) {
        keys.emplace_back(el.first);
    }
    return keys;
}

size_t FileInfo::frames_count() const { return cmap_.size(); }

const FileInfo::frames_map_t& FileInfo::frames_map() const { return cmap_; }

std::string FileInfo::to_string() const {
    std::stringstream ss;
    ss << "FileInfo [id = " << id() << ", "
       << "start_ts = " << start_ts().count() << ", "
       << "end_ts = " << end_ts().count() << ", "
       << "range_multiplier = " << static_cast<int>(range_multiplier()) << ", "
       << "lidar_frame_mode = " << EnumNamesOSF_FRAME_MODE()[lidar_frame_mode()]
       << ", "
       << "mode = " << EnumNamesOSF_SESSION_MODE()[mode()] << "]";
    return ss.str();
}

FileInfo::sensors_map FileInfo::read_sensors() {
    auto osf_session = get_osf_session_from_buf(buf_);
    sensors_map res;
    if (osf_session->sensors() == nullptr) return res;
    for (const auto s : *osf_session->sensors()) {
        res[s->id()] = sensor_from_osf_sensor(s);
    }
    return res;
}

}  // namespace OSF
}  // namespace ouster