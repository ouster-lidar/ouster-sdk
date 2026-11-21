#include <ouster/core/attribute_layout.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {

namespace {

// Converts a double back into an integer attribute. Averaging modes hand us
// fractional values, so round to nearest and clamp instead of letting the
// conversion wrap or trap.
template <typename T>
T from_double(double value, std::true_type /*is_integral*/) {
    if (!(value == value)) {  // NaN
        return T{0};
    }
    const double rounded = std::round(value);
    const double low = static_cast<double>(std::numeric_limits<T>::min());
    // For 64-bit types max() is not exactly representable as a double; the
    // conversion rounds up, so compare against the rounded-down neighbour.
    const double high = static_cast<double>(std::numeric_limits<T>::max());
    if (rounded <= low) {
        return std::numeric_limits<T>::min();
    }
    if (rounded >= high) {
        return std::numeric_limits<T>::max();
    }
    return static_cast<T>(rounded);
}

template <typename T>
T from_double(double value, std::false_type /*is_integral*/) {
    return static_cast<T>(value);
}

template <typename T>
void store(double value, std::uint8_t* dst) {
    const T typed = from_double<T>(value, std::is_integral<T>{});
    std::memcpy(dst, &typed, sizeof(T));
}

template <typename T>
double load(const std::uint8_t* src) {
    T typed;
    std::memcpy(&typed, src, sizeof(T));
    return static_cast<double>(typed);
}

}  // namespace

std::size_t attribute_type_size(AttributeType type) {
    return visit_attribute_type(type, [](auto sample) { return sizeof(sample); });
}

std::string attribute_type_name(AttributeType type) {
    return visit_attribute_type(type, [](auto sample) {
        using T = decltype(sample);
        // Eigen::half (F16) is a class type, so neither is_floating_point nor
        // is_signed sees it as the float it represents.
        constexpr bool is_float =
            std::is_floating_point<T>::value || std::is_same<T, Eigen::half>::value;
        const char kind = is_float ? 'f' : std::is_signed<T>::value ? 'i' : 'u';
        return kind + std::to_string(sizeof(T) * 8);
    });
}

AttributeLayout::AttributeLayout(std::vector<AttributeSpec> specs) : specs_(std::move(specs)) {
    byte_offsets_.reserve(specs_.size());
    element_offsets_.reserve(specs_.size());
    for (const AttributeSpec& spec : specs_) {
        if (spec.count == 0) {
            throw std::invalid_argument("AttributeLayout: attribute count must be greater than 0");
        }
        byte_offsets_.push_back(stride_);
        element_offsets_.push_back(element_count_);
        stride_ += attribute_type_size(spec.type) * spec.count;
        element_count_ += spec.count;
        if (spec.type != AttributeType::F64) {
            all_double_ = false;
        }
    }
}

AttributeLayout AttributeLayout::all_double(std::size_t element_count) {
    std::vector<AttributeSpec> specs(element_count, AttributeSpec{AttributeType::F64, 1});
    return AttributeLayout(std::move(specs));
}

void AttributeLayout::encode(const double* elements, std::uint8_t* record) const {
    if (all_double_) {
        std::memcpy(record, elements, element_count_ * sizeof(double));
        return;
    }
    for (std::size_t i = 0; i < specs_.size(); ++i) {
        const AttributeSpec& spec = specs_[i];
        const double* src = elements + element_offsets_[i];
        std::uint8_t* dst = record + byte_offsets_[i];
        visit_attribute_type(spec.type, [&](auto sample) {
            using T = decltype(sample);
            for (std::uint32_t elem = 0; elem < spec.count; ++elem) {
                store<T>(src[elem], dst + elem * sizeof(T));
            }
        });
    }
}

void AttributeLayout::decode(const std::uint8_t* record, double* elements) const {
    if (all_double_) {
        std::memcpy(elements, record, element_count_ * sizeof(double));
        return;
    }
    for (std::size_t i = 0; i < specs_.size(); ++i) {
        const AttributeSpec& spec = specs_[i];
        const std::uint8_t* src = record + byte_offsets_[i];
        double* dst = elements + element_offsets_[i];
        visit_attribute_type(spec.type, [&](auto sample) {
            using T = decltype(sample);
            for (std::uint32_t elem = 0; elem < spec.count; ++elem) {
                dst[elem] = load<T>(src + elem * sizeof(T));
            }
        });
    }
}

void AttributeLayout::decode_add(const std::uint8_t* record, double* elements) const {
    for (std::size_t i = 0; i < specs_.size(); ++i) {
        const AttributeSpec& spec = specs_[i];
        const std::uint8_t* src = record + byte_offsets_[i];
        double* dst = elements + element_offsets_[i];
        visit_attribute_type(spec.type, [&](auto sample) {
            using T = decltype(sample);
            for (std::uint32_t elem = 0; elem < spec.count; ++elem) {
                dst[elem] += load<T>(src + elem * sizeof(T));
            }
        });
    }
}

bool AttributeLayout::operator==(const AttributeLayout& other) const {
    if (specs_.size() != other.specs_.size()) {
        return false;
    }
    for (std::size_t i = 0; i < specs_.size(); ++i) {
        if (specs_[i].type != other.specs_[i].type || specs_[i].count != other.specs_[i].count) {
            return false;
        }
    }
    return true;
}

void AttributeLayout::check_type(std::size_t spec_index, AttributeType type) const {
    if (spec_index >= specs_.size()) {
        throw std::invalid_argument("spec_index " + std::to_string(spec_index) +
                                    " out of range for a layout of " +
                                    std::to_string(specs_.size()) + " attribute(s)");
    }
    const AttributeSpec& spec = specs_[spec_index];
    if (spec.type != type) {
        throw std::invalid_argument("attribute " + std::to_string(spec_index) + " is stored as " +
                                    std::to_string(spec.count) + " x " +
                                    attribute_type_name(spec.type) + ", not " +
                                    attribute_type_name(type));
    }
}

void AttributeLayout::check_attribute(std::size_t spec_index, AttributeType type,
                                      std::size_t count) const {
    check_type(spec_index, type);
    const AttributeSpec& spec = specs_[spec_index];
    if (spec.count != count) {
        throw std::invalid_argument("attribute " + std::to_string(spec_index) + " is stored as " +
                                    std::to_string(spec.count) + " x " +
                                    attribute_type_name(spec.type) + ", not " +
                                    std::to_string(count) + " x " + attribute_type_name(type));
    }
}

namespace {
// Checks the arguments shared by pack_values() and unpack_values(), once
// check_type() has accepted spec_index: records are stride bytes wide, and
// the typed buffer holds count values for every record.
void check_records_and_count(const char* caller, const AttributeLayout& layout,
                             std::size_t spec_index, Eigen::Index rows, Eigen::Index cols,
                             std::size_t value_count) {
    if (static_cast<std::size_t>(cols) != layout.stride()) {
        throw std::invalid_argument(std::string(caller) + ": records are " + std::to_string(cols) +
                                    " bytes wide, expected " + std::to_string(layout.stride()) +
                                    " (the layout stride)");
    }
    const std::size_t count = layout.specs()[spec_index].count;
    const std::size_t expected = static_cast<std::size_t>(rows) * count;
    if (value_count != expected) {
        throw std::invalid_argument(std::string(caller) + ": got " + std::to_string(value_count) +
                                    " value(s) for attribute " + std::to_string(spec_index) +
                                    ", expected " + std::to_string(expected) + " (" +
                                    std::to_string(rows) + " record(s) x " + std::to_string(count) +
                                    ")");
    }
}
}  // namespace

void AttributeLayout::pack_values(std::size_t spec_index, AttributeType type, const void* values,
                                  std::size_t value_count,
                                  Eigen::Ref<AttributeRecords> records) const {
    check_type(spec_index, type);
    check_records_and_count("AttributeLayout::pack", *this, spec_index, records.rows(),
                            records.cols(), value_count);
    const std::size_t width = byte_size(spec_index);
    const std::size_t offset = byte_offset(spec_index);
    const auto* src = static_cast<const std::uint8_t*>(values);
    // Records are contiguous per row but may be a view with a wider row
    // pitch, so address each row rather than assuming a packed buffer.
    for (Eigen::Index row = 0; row < records.rows(); ++row) {
        std::memcpy(records.row(row).data() + offset, src + static_cast<std::size_t>(row) * width,
                    width);
    }
}

void AttributeLayout::unpack_values(std::size_t spec_index, AttributeType type,
                                    const Eigen::Ref<const AttributeRecords>& records, void* values,
                                    std::size_t value_count) const {
    check_type(spec_index, type);
    check_records_and_count("AttributeLayout::unpack", *this, spec_index, records.rows(),
                            records.cols(), value_count);
    const std::size_t width = byte_size(spec_index);
    const std::size_t offset = byte_offset(spec_index);
    auto* dst = static_cast<std::uint8_t*>(values);
    for (Eigen::Index row = 0; row < records.rows(); ++row) {
        std::memcpy(dst + static_cast<std::size_t>(row) * width, records.row(row).data() + offset,
                    width);
    }
}

std::size_t AttributedPointCloud::size() const {
    return static_cast<std::size_t>(xyz.rows());
}

void AttributedPointCloud::check_shape() const {
    if (attributes.rows() != xyz.rows()) {
        throw std::invalid_argument("AttributedPointCloud: " + std::to_string(xyz.rows()) +
                                    " position(s) but " + std::to_string(attributes.rows()) +
                                    " attribute record(s)");
    }
    if (static_cast<std::size_t>(attributes.cols()) != layout.stride()) {
        throw std::invalid_argument("AttributedPointCloud: records are " +
                                    std::to_string(attributes.cols()) + " bytes wide, expected " +
                                    std::to_string(layout.stride()) + " (the layout stride)");
    }
}

void AttributedPointCloud::copy_records(void* out, std::size_t struct_size) const {
    check_shape();
    if (struct_size != layout.stride()) {
        throw std::invalid_argument(
            "attributes_as: struct is " + std::to_string(struct_size) +
            " bytes but each record is " + std::to_string(layout.stride()) +
            "; records are unpadded, so the struct must be declared packed");
    }
    if (attributes.size() > 0) {
        // attributes is a dense row-major matrix, one record per row, and
        // check_shape() guarantees size() * stride bytes of it.
        std::memcpy(out, attributes.data(), static_cast<std::size_t>(attributes.size()));
    }
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
