#include "ouster/core/field_type.h"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "ouster/core/chanfield.h"
#include "ouster/core/field.h"

namespace ouster {
namespace sdk {
namespace core {

FieldType::FieldType() = default;

FieldType::FieldType(std::string name, ChanFieldType element_type, std::vector<size_t> extra_dims,
                     FieldClass field_class)
    : name(std::move(name)),
      element_type(element_type),
      extra_dims(std::move(extra_dims)),
      field_class(field_class) {}

bool operator==(const FieldType& a, const FieldType& b) {
    return a.name == b.name && a.element_type == b.element_type && a.field_class == b.field_class &&
           a.extra_dims == b.extra_dims;
}

bool operator!=(const FieldType& a, const FieldType& b) {
    return a.name != b.name || a.element_type != b.element_type || a.field_class != b.field_class ||
           a.extra_dims != b.extra_dims;
}

FieldType get_field_type(const std::string& name, const Field& field) {
    int offset = 0;
    if (field.field_class() == FieldClass::PIXEL_FIELD) {
        offset = 2;
    } else if ((field.field_class() == FieldClass::COLUMN_FIELD) ||
               (field.field_class() == FieldClass::PACKET_FIELD)) {
        offset = 1;
    }
    std::vector<size_t> extra_dims;
    extra_dims.insert(extra_dims.begin(), field.shape().begin() + offset, field.shape().end());
    return FieldType(name, field.tag(), extra_dims, field.field_class());
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
