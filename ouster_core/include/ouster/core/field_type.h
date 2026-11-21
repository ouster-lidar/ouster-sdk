#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "ouster/core/chanfield.h"
#include "ouster/core/field.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * Description for a field that we desire in a lidar frame
 */
struct OUSTER_API_CLASS FieldType {
    std::string name;                                  ///< Name of the field
    ChanFieldType element_type{};                      ///< Type of field elements
    std::vector<size_t> extra_dims;                    ///< Additional dimensions of the field
    FieldClass field_class = FieldClass::PIXEL_FIELD;  ///< Category of field, determines the
                                                       ///< first dimensions of the field

    /**
     * Initialize a default FieldType with no name.
     */
    OUSTER_API_FUNCTION
    FieldType();

    /**
     * Initialize a lidar frame from another with only the indicated fields.
     * Casts, zero pads or removes fields from the original frame if necessary.
     *
     * @param[in] name Name of the field described by the type.
     * @param[in] element_type Primitive type for elements in the field.
     * @param[in] extra_dims Additional dimensions of the field
     * @param[in] field_class Category of field, determines the first dimensions
                         of the field
     */
    OUSTER_API_FUNCTION
    FieldType(std::string name, ChanFieldType element_type, std::vector<size_t> extra_dims = {},
              FieldClass field_class = FieldClass::PIXEL_FIELD);

    /**
     * Less than operated needed to allow sorting FieldTypes by name
     *
     * @param[in] other The FieldType to compare with
     *
     * @return if the name is "less than" the provided FieldType
     */
    OUSTER_API_FUNCTION
    inline bool operator<(const FieldType& other) const {
        return name < other.name;
    }
};

/**
 * Get string representation of a FieldType.
 *
 * @param[in] field_type The field type to get the string representation of.
 *
 * @return string representation of the FieldType.
 */
OUSTER_API_FUNCTION
std::string to_string(const FieldType& field_type);

/**
 * Equality for FieldTypes.
 *
 * @param[in] a The first type to compare.
 * @param[in] b The second type to compare.
 *
 * @return if a == b.
 */
OUSTER_API_FUNCTION
bool operator==(const FieldType& a, const FieldType& b);

/**
 * Equality for FieldTypes.
 *
 * @param[in] a The first type to compare.
 * @param[in] b The second type to compare.
 *
 * @return if a != b.
 */
OUSTER_API_FUNCTION
bool operator!=(const FieldType& a, const FieldType& b);

/**
 * Alias for the lidar frame field types
 */
using LidarFrameFieldTypes = std::vector<FieldType>;

/**
 * Build a FieldType descriptor from an existing Field and its name.
 *
 * @param[in] name Field name to store in the returned descriptor.
 * @param[in] field Field whose element type, shape, and class are copied.
 *
 * @return FieldType describing @p name and @p field.
 */
OUSTER_API_FUNCTION
FieldType get_field_type(const std::string& name, const Field& field);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
