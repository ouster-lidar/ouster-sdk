#include "ouster/core/frame.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <nonstd/optional.hpp>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster/core/chanfield.h"
#include "ouster/core/field.h"
#include "ouster/core/field_type.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/object.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/types.h"

namespace ouster {
namespace sdk {
namespace core {

Frame::~Frame() = default;

const std::shared_ptr<SensorInfo>& Frame::sensor_info() const {
    return sensor_info_;
}

bool Frame::accepts_sensor_info(const SensorInfo& /*info*/) const {
    return true;
}

namespace impl {

void assign_sensor_info(Frame& frame, std::shared_ptr<SensorInfo> info) {
    if (info && !frame.accepts_sensor_info(*info)) {
        throw std::invalid_argument("sensor info does not describe this frame's sensor kind");
    }
    frame.sensor_info_ = std::move(info);
}

}  // namespace impl

const std::unordered_map<std::string, Field>& Frame::fields() const {
    return fields_;
}

Field& Frame::field(const std::string& name) {
    try {
        return fields().at(name);
    } catch (std::out_of_range& e) {
        throw std::out_of_range("Field '" + name + "' not found in frame.");
    }
}

const Field& Frame::field(const std::string& name) const {
    try {
        return fields().at(name);
    } catch (std::out_of_range& e) {
        throw std::out_of_range("Field '" + name + "' not found in frame.");
    }
}

bool Frame::has_field(const std::string& name) const {
    return fields().count(name) > 0;
}

FieldDescriptor Frame::get_field_type_descriptor(const FieldType& field_type) const {
    if (field_type.field_class == FieldClass::PIXEL_FIELD) {
        std::vector<size_t> dims;
        dims.push_back(h);
        dims.push_back(w);
        dims.insert(dims.end(), field_type.extra_dims.begin(), field_type.extra_dims.end());
        return FieldDescriptor::array(field_type.element_type, dims);
    } else if (field_type.field_class == FieldClass::COLUMN_FIELD) {
        std::vector<size_t> dims;
        dims.push_back(w);
        dims.insert(dims.end(), field_type.extra_dims.begin(), field_type.extra_dims.end());
        return FieldDescriptor::array(field_type.element_type, dims);
    } else if (field_type.field_class == FieldClass::PACKET_FIELD) {
        std::vector<size_t> dims;
        dims.push_back(packet_count());
        dims.insert(dims.end(), field_type.extra_dims.begin(), field_type.extra_dims.end());
        return FieldDescriptor::array(field_type.element_type, dims);
    } else {  // FieldClass::FRAME_FIELD
        return FieldDescriptor::array(field_type.element_type, field_type.extra_dims);
    }
}

Field& Frame::add_field(const FieldType& type, bool zero) {
    if (has_field(type.name)) {
        throw std::invalid_argument("Duplicated field '" + type.name + "'");
    }

    // just validate that we didn't add a 0 size pixel field
    if (type.field_class == FieldClass::PIXEL_FIELD) {
        // none of the dimensions should be zero
        for (const auto& dim : type.extra_dims) {
            if (dim == 0) {
                throw std::invalid_argument("Cannot add pixel field with 0 elements.");
            }
        }
    }

    // no other checking is necessary
    fields()[type.name] = Field(get_field_type_descriptor(type), type.field_class, zero);

    return fields()[type.name];
}

Field& Frame::add_field(const std::string& name, FieldDescriptor desc, FieldClass field_class,
                        bool zero) {
    if (has_field(name)) {
        throw std::invalid_argument("Duplicated field '" + name + "'");
    }

    if (field_class == FieldClass::PIXEL_FIELD) {
        if (desc.shape.size() < 2) {
            throw std::invalid_argument("Pixel fields must have at least 2 dimensions");
        }
        if (desc.shape[0] != h || desc.shape[1] != w) {
            throw std::invalid_argument(
                "Pixel field shape must match "
                "frame's width and height. Was " +
                std::to_string(desc.shape[0]) + "x" + std::to_string(desc.shape[1]) + " vs " +
                std::to_string(h) + "x" + std::to_string(w));
        }
        // none of the dimensions should be zero
        for (const auto& dim : desc.shape) {
            if (dim == 0) {
                throw std::invalid_argument("Cannot add pixel field with 0 elements.");
            }
        }
    }

    if (field_class == FieldClass::COLUMN_FIELD) {
        if (desc.shape[0] != w) {
            throw std::invalid_argument(
                "Column field shape must match "
                "frame's height. Width was " +
                std::to_string(desc.shape[0]) + " vs required width of " + std::to_string(w));
        }
    }

    if (field_class == FieldClass::PACKET_FIELD) {
        if (desc.shape[0] != packet_count_) {
            throw std::invalid_argument(
                "Packet field shape must match "
                "number of packets. Width was " +
                std::to_string(desc.shape[0]) + " vs required width of " +
                std::to_string(packet_count_));
        }
    }

    fields()[name] = Field{desc, field_class, zero};

    return fields()[name];
}

Field Frame::del_field(const std::string& name) {
    if (!has_field(name)) {
        throw std::invalid_argument("Attempted deleting non existing field '" + name + "'");
    }

    Field ptr;
    field(name).swap(ptr);
    fields().erase(name);
    return ptr;
}

// Frame::field<T>(...) is defined inline in lidar_scan.h to work around
// an Apple Clang mangling bug for the dependent default StrideType expression
// in Eigen::Ref. See the note in lidar_scan.h. (float16_t and other types are
// covered by implicit instantiation wherever they are used.)

namespace {}  // namespace

FieldType Frame::field_type(const std::string& name) const {
    return get_field_type(name, field(name));
}

std::unordered_map<std::string, Field>& Frame::fields() {
    return fields_;
}

std::unordered_map<std::string, std::vector<Object>>& Frame::objects() {
    return objects_;
}

const std::unordered_map<std::string, std::vector<Object>>& Frame::objects() const {
    return objects_;
}

size_t Frame::packet_count() const {
    return packet_count_;
}

void Frame::init_timestamps_and_poses(size_t count, FieldClass timestamp_class,
                                      FieldClass pose_class) {
    timestamp_ = Field{fd_array<uint64_t>(count), timestamp_class};
    body_to_world_ = Field{fd_array<double>(count, 4, 4), pose_class};

    auto ptr = body_to_world_.get<double>();
    auto ident = mat4d::Identity().eval();
    for (size_t i = 0; i < count; ++i) {
        std::memcpy(&ptr[i * 4 * 4], ident.data(), static_cast<size_t>(4 * 4) * sizeof(double));
    }
}

Eigen::Ref<Frame::Header<uint64_t>> Frame::timestamp() {
    return timestamp_;
}

Eigen::Ref<const Frame::Header<uint64_t>> Frame::timestamp() const {
    return timestamp_;
}

Eigen::Ref<Frame::Header<uint64_t>> Frame::receive_timestamp() {
    return receive_timestamp_;
}

Eigen::Ref<const Frame::Header<uint64_t>> Frame::receive_timestamp() const {
    return receive_timestamp_;
}

Eigen::Ref<Frame::Header<uint32_t>> Frame::status() {
    return status_;
}

Eigen::Ref<const Frame::Header<uint32_t>> Frame::status() const {
    return status_;
}

namespace {

uint64_t ts_value_or_throw(nonstd::optional<uint64_t> time) {
    if (!time) {
        throw std::runtime_error("No valid packets in Frame");
    }
    return *time;
}

// Number of status entries mapped to each packet. Returns 0 when the frame has
// no packets or no status (in which case there are no valid packet timestamps).
int columns_per_packet_or_zero(Eigen::Index status_size, Eigen::Index packet_count) {
    if (packet_count <= 0 || status_size <= 0) {
        return 0;
    }
    return static_cast<int>(status_size / packet_count);
}

}  // namespace

nonstd::optional<uint64_t> Frame::first_valid_receive_timestamp_internal() const {
    const auto stat = status();
    const auto pts = receive_timestamp();
    const int total_packets = static_cast<int>(pts.size());
    const int columns_per_packet = columns_per_packet_or_zero(stat.size(), pts.size());
    if (columns_per_packet == 0) {
        return nonstd::nullopt;
    }
    for (int i = 0; i < total_packets; ++i) {
        if (stat.middleRows(static_cast<Eigen::Index>(i) * columns_per_packet, columns_per_packet)
                .unaryExpr([](uint32_t status_val) { return status_val & 1u; })
                .any()) {
            return pts[i];
        }
    }
    return nonstd::nullopt;
}

nonstd::optional<uint64_t> Frame::last_valid_receive_timestamp_internal() const {
    const auto stat = status();
    const auto pts = receive_timestamp();
    const int total_packets = static_cast<int>(pts.size());
    const int columns_per_packet = columns_per_packet_or_zero(stat.size(), pts.size());
    if (columns_per_packet == 0) {
        return nonstd::nullopt;
    }
    for (int i = total_packets - 1; i >= 0; --i) {
        if (stat.middleRows(static_cast<Eigen::Index>(i) * columns_per_packet, columns_per_packet)
                .unaryExpr([](uint32_t status_val) { return status_val & 1u; })
                .any()) {
            return pts[i];
        }
    }
    return nonstd::nullopt;
}

nonstd::optional<uint64_t> Frame::min_valid_receive_timestamp_internal() const {
    const auto stat = status();
    const auto pts = receive_timestamp();
    const int total_packets = static_cast<int>(pts.size());
    const int columns_per_packet = columns_per_packet_or_zero(stat.size(), pts.size());
    if (columns_per_packet == 0) {
        return nonstd::nullopt;
    }
    nonstd::optional<uint64_t> time;
    for (int i = 0; i < total_packets; ++i) {
        if (stat.middleRows(static_cast<Eigen::Index>(i) * columns_per_packet, columns_per_packet)
                .unaryExpr([](uint32_t status_val) { return status_val & 1u; })
                .any()) {
            const uint64_t packet_time = pts[i];
            time = time ? std::min(*time, packet_time) : packet_time;
        }
    }
    return time;
}

nonstd::optional<uint64_t> Frame::max_valid_receive_timestamp_internal() const {
    const auto stat = status();
    const auto pts = receive_timestamp();
    const int total_packets = static_cast<int>(pts.size());
    const int columns_per_packet = columns_per_packet_or_zero(stat.size(), pts.size());
    if (columns_per_packet == 0) {
        return nonstd::nullopt;
    }
    nonstd::optional<uint64_t> time;
    for (int i = 0; i < total_packets; ++i) {
        if (stat.middleRows(static_cast<Eigen::Index>(i) * columns_per_packet, columns_per_packet)
                .unaryExpr([](uint32_t status_val) { return status_val & 1u; })
                .any()) {
            const uint64_t packet_time = pts[i];
            time = time ? std::max(*time, packet_time) : packet_time;
        }
    }
    return time;
}

uint64_t Frame::get_first_valid_receive_timestamp() const {
    return ts_value_or_throw(first_valid_receive_timestamp_internal());
}

uint64_t Frame::get_last_valid_receive_timestamp() const {
    return ts_value_or_throw(last_valid_receive_timestamp_internal());
}

uint64_t Frame::get_min_valid_receive_timestamp() const {
    return ts_value_or_throw(min_valid_receive_timestamp_internal());
}

uint64_t Frame::get_max_valid_receive_timestamp() const {
    return ts_value_or_throw(max_valid_receive_timestamp_internal());
}

int Frame::get_first_valid_column() const {
    auto stat = status();
    for (int i = 0; i < stat.size(); ++i) {
        if ((stat[i] & 1) > 0) {
            return i;
        }
    }
    throw std::runtime_error("No valid columns in Frame");
}

int Frame::get_last_valid_column() const {
    auto stat = status();
    for (int i = static_cast<int>(stat.size()) - 1; i >= 0; --i) {
        if ((stat[i] & 1) > 0) {
            return i;
        }
    }
    throw std::runtime_error("No valid columns in Frame");
}

Field& Frame::body_to_world() {
    return body_to_world_;
}

const Field& Frame::body_to_world() const {
    return body_to_world_;
}

namespace {

int pose_count(const Field& body_to_world) {
    const auto& shape = body_to_world.shape();
    if (shape.empty()) {
        return 0;
    }
    return static_cast<int>(shape[0]);
}

}  // namespace

void Frame::set_column_pose(int index, const Matrix4dR& pose) {
    if (index < 0 || index >= pose_count(body_to_world_)) {
        throw std::out_of_range("Column index out of range");
    }
    std::memcpy(static_cast<char*>(body_to_world_.get()) +
                    (static_cast<size_t>(index) * 16 * sizeof(double)),
                pose.data(), sizeof(double) * 16);
}

Matrix4dR Frame::get_column_pose(int index) const {
    if (index < 0 || index >= pose_count(body_to_world_)) {
        throw std::out_of_range("Column index out of range");
    }
    Matrix4dR out;
    std::memcpy(out.data(),
                static_cast<const char*>(body_to_world_.get()) +
                    (static_cast<size_t>(index) * 16 * sizeof(double)),
                sizeof(double) * 16);
    return out;
}

namespace {

/// Render a CHAR field's leading bytes as a printable, escaped preview, stopping
/// at the first NUL or after max_len bytes (in which case "..." is appended).
std::string escaped_preview(const char* data, size_t max_len) {
    const char* end = static_cast<const char*>(memchr(data, '\0', max_len));
    size_t len = (end != nullptr) ? static_cast<size_t>(end - data) : max_len;

    std::string preview;
    preview.reserve(len);
    for (size_t i = 0; i < len; ++i) {
        switch (data[i]) {
            case '\r':
                preview += "\\r";
                break;
            case '\n':
                preview += "\\n";
                break;
            case '\t':
                preview += "\\t";
                break;
            case '\\':
                preview += "\\\\";
                break;
            case '\b':
                preview += "\\b";
                break;
            case '\f':
                preview += "\\f";
                break;
            case '\v':
                preview += "\\v";
                break;
            case '"':
                preview += "\\\"";
                break;
            default:
                preview += data[i];
                break;
        }
    }

    if (end == nullptr && len == max_len) {
        preview += "...";
    }
    return preview;
}

}  // namespace

namespace impl {

void append_frame_fields(const std::unordered_map<std::string, Field>& fields,
                         std::ostream& out_stream) {
    auto read_eigen = [](auto ref, std::ostream& out) {
        out << "min: " << static_cast<double>(ref.minCoeff())
            << "; mean: " << ref.template cast<double>().mean()
            << "; max: " << static_cast<double>(ref.maxCoeff());
    };

    auto read_field = [&read_eigen](const Field& field_val, const std::string& name,
                                    std::ostream& out) {
        out << "     " << name << " type:" << to_string(field_val.tag()) << " shape: (";

        const auto& shape = field_val.shape();
        for (size_t i = 0; i < shape.size(); ++i) {
            out << shape[i];
            if (i < shape.size() - 1) {
                out << ", ";
            }
        }
        out << ") ";

        if (field_val.bytes() > 0) {
            if (field_val.tag() == ChanFieldType::CHAR) {
                // Show first sentence (e.g., first NMEA string).
                size_t max_len = NMEA_SENTENCE_LENGTH;
                const auto& char_shape = field_val.shape();
                if (!char_shape.empty() && char_shape.back() > 0) {
                    max_len = char_shape.back();
                }
                out << "\"" << escaped_preview(static_cast<const char*>(field_val.get()), max_len)
                    << "\"";
            } else {
                // For numeric types, show min/mean/max stats.
                // visit_field_2d will no-op for unsupported types (ZONE_STATE,
                // etc.)
                FieldView flat_view = field_val.reshape(1, field_val.size());
                impl::visit_field_2d(flat_view, read_eigen, out);
            }
        }
        out << std::endl;
    };

    for (auto&& kv : fields) {
        read_field(kv.second, kv.first, out_stream);
    }
}

}  // namespace impl

std::string Frame::to_string() const {
    std::stringstream string_stream;
    string_stream << "Frame: {h = " << h << ", w = " << w << "," << std::endl;
    impl::append_frame_fields(fields_, string_stream);
    string_stream << "}";
    return string_stream.str();
}

LidarFrameFieldTypes Frame::field_types() const {
    LidarFrameFieldTypes field_types_vec;
    for (const auto& kv : fields()) {
        field_types_vec.push_back(get_field_type(kv.first, kv.second));
    }

    std::sort(field_types_vec.begin(), field_types_vec.end());
    return field_types_vec;
}

bool Frame::equals(const Frame& other) const {
    // Frames of different concrete types are never equal. Establishing that
    // here also lets derived overrides static_cast<> their operand safely.
    return typeid(*this) == typeid(other) && w == other.w && h == other.h &&
           packet_count_ == other.packet_count_ && timestamp_ == other.timestamp_ &&
           receive_timestamp_ == other.receive_timestamp_ && status_ == other.status_ &&
           body_to_world_ == other.body_to_world_ && fields_ == other.fields_ &&
           objects_ == other.objects_;
}

std::string to_string(const Frame& frame) {
    return frame.to_string();
}

bool operator==(const Frame& a, const Frame& b) {
    return a.equals(b);
}

bool operator!=(const Frame& a, const Frame& b) {
    return !a.equals(b);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
