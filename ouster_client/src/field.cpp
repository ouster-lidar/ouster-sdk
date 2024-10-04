/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/field.h"

#include <cstring>

namespace ouster {

namespace impl {

std::vector<size_t> calculate_strides(const std::vector<size_t>& shape) {
    auto total = std::accumulate(shape.begin(), shape.end(), size_t{1},
                                 std::multiplies<size_t>{});
    auto strides = std::vector<size_t>{};
    strides.reserve(shape.size());
    for (auto dim : shape) {
        if (dim == 0) {
            strides.push_back(1);
            continue;
        }
        total /= dim;
        strides.push_back(total);
    }
    return strides;
}

}  // namespace impl

namespace sensor {
namespace impl {

// clang-format off

template <> int type_size<void>() { return 1; }

template <> size_t type_hash<void>() { return 0; }

template <> ChanFieldType type_cft<void>() { return ChanFieldType::VOID; }
template <> ChanFieldType type_cft<uint8_t>() { return ChanFieldType::UINT8; }
template <> ChanFieldType type_cft<uint16_t>() { return ChanFieldType::UINT16; }
template <> ChanFieldType type_cft<uint32_t>() { return ChanFieldType::UINT32; }
template <> ChanFieldType type_cft<uint64_t>() { return ChanFieldType::UINT64; }
template <> ChanFieldType type_cft<int8_t>() { return ChanFieldType::INT8; }
template <> ChanFieldType type_cft<int16_t>() { return ChanFieldType::INT16; }
template <> ChanFieldType type_cft<int32_t>() { return ChanFieldType::INT32; }
template <> ChanFieldType type_cft<int64_t>() { return ChanFieldType::INT64; }
template <> ChanFieldType type_cft<float>() { return ChanFieldType::FLOAT32; }
template <> ChanFieldType type_cft<double>() { return ChanFieldType::FLOAT64; }

// clang-format on

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

extern const Table<FieldClass, const char*, 4> field_class_strings{
    {{FieldClass::PIXEL_FIELD, "PIXEL_FIELD"},
     {FieldClass::COLUMN_FIELD, "COLUMN_FIELD"},
     {FieldClass::PACKET_FIELD, "PACKET_FIELD"},
     {FieldClass::SCAN_FIELD, "SCAN_FIELD"}}};

}  // namespace impl
}  // namespace sensor

std::string to_string(FieldClass flag) {
    auto end = sensor::impl::field_class_strings.end();
    auto res = std::find_if(sensor::impl::field_class_strings.begin(), end,
                            [flag](const auto& p) { return p.first == flag; });

    return res == end ? "UNKNOWN" : res->second;
}

size_t FieldDescriptor::size() const {
    return std::accumulate(shape.begin(), shape.end(), size_t{1},
                           std::multiplies<size_t>{});
}

size_t FieldDescriptor::bytes() const { return size() * element_size; }

sensor::ChanFieldType FieldDescriptor::tag() const {
    using sensor::impl::type_cft;

    if (type == type_hash<uint8_t>()) {
        return type_cft<uint8_t>();
    } else if (type == type_hash<uint16_t>()) {
        return type_cft<uint16_t>();
    } else if (type == type_hash<uint32_t>()) {
        return type_cft<uint32_t>();
    } else if (type == type_hash<uint64_t>()) {
        return type_cft<uint64_t>();
    } else if (type == type_hash<int8_t>()) {
        return type_cft<int8_t>();
    } else if (type == type_hash<int16_t>()) {
        return type_cft<int16_t>();
    } else if (type == type_hash<int32_t>()) {
        return type_cft<int32_t>();
    } else if (type == type_hash<int64_t>()) {
        return type_cft<int64_t>();
    } else if (type == type_hash<float>()) {
        return type_cft<float>();
    } else if (type == type_hash<double>()) {
        return type_cft<double>();
    } else if (type == type_hash<void>()) {
        return type_cft<void>();
    } else {
        return sensor::ChanFieldType::UNREGISTERED;
    }
}

void FieldDescriptor::swap(FieldDescriptor& other) {
    std::swap(type, other.type);
    std::swap(shape, other.shape);
    std::swap(strides, other.strides);
    std::swap(element_size, other.element_size);
}

bool FieldDescriptor::is_type_compatible(
    const FieldDescriptor& other) const noexcept {
    return !type || !other.type || type == other.type;
}

size_t FieldDescriptor::ndim() const noexcept { return shape.size(); }

FieldView::FieldView() noexcept : ptr_(nullptr), desc_() {}

FieldView::FieldView(void* ptr, const FieldDescriptor& desc)
    : ptr_(ptr), desc_(desc) {}

FieldView::operator bool() const noexcept { return !!get(); }

size_t FieldView::bytes() const noexcept { return desc_.bytes(); }

size_t FieldView::size() const { return desc_.size(); }

bool FieldView::matches(const FieldDescriptor& d) const noexcept {
    return desc_ == d;
}

const FieldDescriptor& FieldView::desc() const { return desc_; }

const std::vector<size_t>& FieldView::shape() const { return desc_.shape; }

sensor::ChanFieldType FieldView::tag() const { return desc_.tag(); }

bool FieldView::sparse() const {
    const auto& strides = desc_.strides;
    const auto& shape = desc_.shape;

    if (shape.size() == 0) return false;

    bool dense = strides.back() == 1;
    for (int i = 1, end = shape.size(); i < end; ++i) {
        dense = dense && (strides[i - 1] == strides[i] * shape[i]);
    }
    return !dense;
}

Field::Field() noexcept : FieldView(), class_{FieldClass::SCAN_FIELD} {}
Field::~Field() { free(ptr_); }

Field::Field(const FieldDescriptor& desc, FieldClass field_class)
    : FieldView(nullptr, desc), class_{field_class} {
    ptr_ = calloc(desc.bytes(), sizeof(uint8_t));
    if (!ptr_) {
        throw std::runtime_error("Field: host allocation failed");
    }
}

Field::Field(Field&& other) noexcept : Field() { swap(other); };

Field& Field::operator=(Field&& other) noexcept {
    swap(other);
    return (*this);
}

Field::Field(const Field& other)
    : FieldView(nullptr, other.desc()), class_{other.class_} {
    ptr_ = malloc(desc().bytes());
    if (!ptr_) {
        throw std::runtime_error("Field: host allocation failed");
    }
    std::memcpy(ptr_, other.ptr_, other.bytes());
}

Field& Field::operator=(const Field& other) {
    Field new_fp(other);
    swap(new_fp);
    return (*this);
}

FieldClass Field::field_class() const { return class_; }

void Field::swap(Field& other) noexcept {
    std::swap(ptr_, other.ptr_);
    desc_.swap(other.desc_);
    std::swap(class_, other.class_);
}

bool Field::operator==(const Field& other) const {
    return matches(other.desc()) &&
           (std::memcmp(ptr_, other.ptr_, bytes()) == 0) &&
           class_ == other.class_;
}

FieldView uint_view(const FieldView& other) {
    if (other.shape().size() == 0) {
        throw std::invalid_argument(
            "uint_view: attempted converting a non-array FieldView");
    }

    FieldDescriptor desc;
    switch (other.desc().element_size) {
        case 1:
            desc = FieldDescriptor::array<uint8_t>(other.shape());
            break;
        case 2:
            desc = FieldDescriptor::array<uint16_t>(other.shape());
            break;
        case 4:
            desc = FieldDescriptor::array<uint32_t>(other.shape());
            break;
        case 8:
            desc = FieldDescriptor::array<uint64_t>(other.shape());
            break;
        default:
            // shape size check should usually suffice, but this may trigger
            // on strange cases like views bound to arrays of custom structs
            throw std::invalid_argument(
                "uint_view: got wrong element size " +
                std::to_string(other.desc().element_size) +
                ", are you using an array "
                "of primitives?");
    }

    return {const_cast<void*>(other.get()), desc};
}

}  // namespace ouster

void std::swap(ouster::Field& a, ouster::Field& b) { a.swap(b); }
