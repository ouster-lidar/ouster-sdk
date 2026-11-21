#pragma once

#include <ouster/core/typedefs.h>
#include <ouster/core/visibility.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {

/// @brief Scalar type of a point attribute stored alongside a point in a
/// @ref ouster::sdk::core::VoxelHashMap.
///
/// The map keeps attributes in the type listed here rather than widening
/// everything to double, so a RANGE (uint32) attribute costs 4 bytes per point
/// and a REFLECTIVITY (uint8) attribute costs 1, and integer values survive the
/// round trip through the map bit-for-bit.
enum class AttributeType : std::uint8_t {
    U8,   ///< std::uint8_t
    U16,  ///< std::uint16_t
    U32,  ///< std::uint32_t
    U64,  ///< std::uint64_t
    I8,   ///< std::int8_t
    I16,  ///< std::int16_t
    I32,  ///< std::int32_t
    I64,  ///< std::int64_t
    F16,  ///< Eigen::half (IEEE 754 binary16, half precision)
    F32,  ///< float
    F64   ///< double
};

/// @brief Every AttributeType, in declaration order.
///
/// Lets code that maps attribute types onto another type system (e.g. numpy
/// dtypes in the Python bindings) enumerate them and derive each mapping via
/// `visit_attribute_type()`, rather than keeping its own table.
constexpr AttributeType ALL_ATTRIBUTE_TYPES[] = {
    AttributeType::U8,  AttributeType::U16, AttributeType::U32, AttributeType::U64,
    AttributeType::I8,  AttributeType::I16, AttributeType::I32, AttributeType::I64,
    AttributeType::F16, AttributeType::F32, AttributeType::F64};

/// @brief Size in bytes of one scalar of @p type.
/// @param[in] type The attribute scalar type.
/// @return Size of one scalar in bytes.
OUSTER_API_FUNCTION std::size_t attribute_type_size(AttributeType type);

/// @brief Short name of @p type, e.g. "u32": its kind (`u`nsigned, `i`nteger
/// or `f`loat) followed by its width in bits, derived from the C++ type
/// `visit_attribute_type()` maps it to.
/// @param[in] type The attribute scalar type.
/// @return The type's short name.
/// @throw std::invalid_argument if @p type is not a known AttributeType.
OUSTER_API_FUNCTION std::string attribute_type_name(AttributeType type);

/// @brief Invoke @p func with a value-initialized instance of the C++ type that
/// @p type names, e.g. `std::uint32_t{}` for AttributeType::U32.
///
/// Keeps the eleven-way switch over AttributeType in one place. @p func must be a
/// generic callable -- a generic lambda, say -- that compiles for every
/// supported scalar type and returns the same type for each of them.
///
/// @param[in] type The attribute scalar type to dispatch on.
/// @param[in] func Callable invoked with one value of the matching type.
/// @return Whatever @p func returns.
/// @throw std::invalid_argument if @p type is not a known AttributeType.
template <typename Func>
auto visit_attribute_type(AttributeType type, Func&& func) -> decltype(func(std::uint8_t{})) {
    switch (type) {
        case AttributeType::U8:
            return func(std::uint8_t{});
        case AttributeType::U16:
            return func(std::uint16_t{});
        case AttributeType::U32:
            return func(std::uint32_t{});
        case AttributeType::U64:
            return func(std::uint64_t{});
        case AttributeType::I8:
            return func(std::int8_t{});
        case AttributeType::I16:
            return func(std::int16_t{});
        case AttributeType::I32:
            return func(std::int32_t{});
        case AttributeType::I64:
            return func(std::int64_t{});
        case AttributeType::F16:
            return func(Eigen::half{});
        case AttributeType::F32:
            return func(float{});
        case AttributeType::F64:
            return func(double{});
    }
    throw std::invalid_argument("AttributeLayout: unknown AttributeType");
}

namespace impl {
/// @brief The AttributeType stored as C++ type @p T, e.g.
/// `AttributeTypeOf<std::uint32_t>::value == AttributeType::U32`; the inverse
/// of `visit_attribute_type()`. Only defined for the eleven supported scalar
/// types, so any other @p T fails to compile.
template <typename T>
struct AttributeTypeOf;
/// @cond
#define OUSTER_ATTRIBUTE_TYPE_OF(cpp_type, attribute_type)                    \
    template <>                                                               \
    struct AttributeTypeOf<cpp_type> {                                        \
        static constexpr AttributeType value = AttributeType::attribute_type; \
    };
OUSTER_ATTRIBUTE_TYPE_OF(std::uint8_t, U8)
OUSTER_ATTRIBUTE_TYPE_OF(std::uint16_t, U16)
OUSTER_ATTRIBUTE_TYPE_OF(std::uint32_t, U32)
OUSTER_ATTRIBUTE_TYPE_OF(std::uint64_t, U64)
OUSTER_ATTRIBUTE_TYPE_OF(std::int8_t, I8)
OUSTER_ATTRIBUTE_TYPE_OF(std::int16_t, I16)
OUSTER_ATTRIBUTE_TYPE_OF(std::int32_t, I32)
OUSTER_ATTRIBUTE_TYPE_OF(std::int64_t, I64)
OUSTER_ATTRIBUTE_TYPE_OF(Eigen::half, F16)
OUSTER_ATTRIBUTE_TYPE_OF(float, F32)
OUSTER_ATTRIBUTE_TYPE_OF(double, F64)
#undef OUSTER_ATTRIBUTE_TYPE_OF
/// @endcond
}  // namespace impl

/// @brief One attribute of a point: @p count scalars of type @p type.
///
/// A scalar field such as RANGE is `{U32, 1}`; a three-channel field such as
/// RGB or NORMALS is `{F32, 3}`.
struct OUSTER_API_CLASS AttributeSpec {
    AttributeType type{AttributeType::F64};  ///< Scalar type of every element.
    std::uint32_t count{1};                  ///< Number of scalars (>= 1).
};

/// @brief Packed attribute records, one row of `AttributeLayout::stride()`
/// bytes per point.
///
/// Taken by @ref ouster::sdk::core::VoxelHashMap::add_points through an
/// `Eigen::Ref`, so the records can be a view over any buffer whose rows are
/// contiguous (an `Eigen::Map` over a std::vector, a numpy array, a block of a
/// larger matrix, ...) without being copied first.
using AttributeRecords = MatrixXXR<std::uint8_t>;

/// @brief Runtime description of the attribute payload a
/// @ref ouster::sdk::core::VoxelHashMap stores next to each point's position.
///
/// A layout turns an ordered list of @ref AttributeSpec into a packed record:
/// every point in the map owns `stride()` contiguous bytes, and attribute
/// `i` occupies `spec.count * attribute_type_size(spec.type)` bytes starting at
/// `byte_offset(i)`. Records are packed without padding, so they are read and
/// written through `std::memcpy` rather than by casting.
///
/// The same layout also describes the double-valued view of a point used by the
/// map's `Eigen`-based API: attribute `i` covers the `count` doubles starting at
/// `element_offset(i)` of the attribute portion of a point (i.e. column
/// `3 + element_offset(i)` of a pointcloud row). @ref encode and @ref decode
/// convert between the two views.
///
/// A default-constructed layout has no attributes.
class OUSTER_API_CLASS AttributeLayout {
   public:
    /// @brief Construct an empty layout (no attributes).
    OUSTER_API_FUNCTION AttributeLayout() = default;

    /// @brief Construct a layout from an ordered list of attributes.
    /// @param[in] specs The attributes, in the order they appear in a record.
    /// @throw std::invalid_argument if any spec has a count of 0.
    OUSTER_API_FUNCTION explicit AttributeLayout(std::vector<AttributeSpec> specs);

    /// @brief Build a layout of @p element_count double attributes.
    ///
    /// @param[in] element_count Number of double attributes.
    /// @return The corresponding layout.
    OUSTER_API_FUNCTION static AttributeLayout all_double(std::size_t element_count);

    /// @brief The attributes making up this layout.
    /// @return The attribute specs, in record order.
    OUSTER_API_FUNCTION const std::vector<AttributeSpec>& specs() const {
        return specs_;
    }

    /// @brief Byte offset of attribute @p spec_index within a packed record.
    /// @param[in] spec_index Index into @ref specs.
    /// @return Offset in bytes from the start of the record.
    /// @throw std::out_of_range if @p spec_index is out of range.
    OUSTER_API_FUNCTION std::size_t byte_offset(std::size_t spec_index) const {
        check_spec_index(spec_index);
        return byte_offsets_[spec_index];
    }

    /// @brief Size in bytes of attribute @p spec_index.
    /// @param[in] spec_index Index into @ref specs.
    /// @return Size in bytes of the attribute's elements.
    /// @throw std::out_of_range if @p spec_index is out of range.
    OUSTER_API_FUNCTION std::size_t byte_size(std::size_t spec_index) const {
        check_spec_index(spec_index);
        const AttributeSpec& spec = specs_[spec_index];
        return attribute_type_size(spec.type) * spec.count;
    }

    /// @brief Index of attribute @p spec_index's first scalar in the
    /// double-valued view of a point's attributes.
    /// @param[in] spec_index Index into @ref specs.
    /// @return Offset in scalars from the start of the attribute block.
    /// @throw std::out_of_range if @p spec_index is out of range.
    OUSTER_API_FUNCTION std::size_t element_offset(std::size_t spec_index) const {
        check_spec_index(spec_index);
        return element_offsets_[spec_index];
    }

    /// @brief Size in bytes of one packed attribute record.
    /// @return The record stride; 0 when the layout has no attributes.
    OUSTER_API_FUNCTION std::size_t stride() const {
        return stride_;
    }

    /// @brief Total number of scalars across all attributes.
    /// @return The number of doubles the attributes occupy in a point vector.
    OUSTER_API_FUNCTION std::size_t element_count() const {
        return element_count_;
    }

    /// @brief Whether the layout has no attributes at all.
    /// @return True when there are no attributes.
    OUSTER_API_FUNCTION bool empty() const {
        return specs_.empty();
    }

    /// @brief Whether every attribute is a double.
    ///
    /// True for empty layouts and for those built by @ref all_double; @ref encode
    /// and @ref decode then reduce to a single `memcpy`.
    /// @return True when no narrowing conversion is involved.
    OUSTER_API_FUNCTION bool all_double() const {
        return all_double_;
    }

    /// @brief Pack @ref element_count doubles into a record of @ref stride bytes.
    ///
    /// Values are converted to each attribute's type; integer attributes are
    /// rounded to nearest and clamped to the type's range, so averaging modes
    /// produce a representable value rather than wrapping.
    /// @param[in]  elements Pointer to @ref element_count doubles.
    /// @param[out] record   Pointer to @ref stride writable bytes.
    OUSTER_API_FUNCTION void encode(const double* elements, std::uint8_t* record) const;

    /// @brief Unpack a record of @ref stride bytes into @ref element_count doubles.
    /// @param[in]  record   Pointer to @ref stride readable bytes.
    /// @param[out] elements Pointer to @ref element_count writable doubles.
    OUSTER_API_FUNCTION void decode(const std::uint8_t* record, double* elements) const;

    /// @brief Add the values in @p record to @p elements element-wise.
    ///
    /// Used by the averaging bucket, which accumulates in double regardless of
    /// the stored attribute types.
    /// @param[in]     record   Pointer to @ref stride readable bytes.
    /// @param[in,out] elements Pointer to @ref element_count doubles to add into.
    OUSTER_API_FUNCTION void decode_add(const std::uint8_t* record, double* elements) const;

    /// @brief Throw unless attribute @p spec_index is stored as @p count
    /// scalars of @p type.
    /// @param[in] spec_index Index into @ref specs.
    /// @param[in] type       The expected scalar type.
    /// @param[in] count      The expected number of scalars per point.
    /// @throw std::invalid_argument if @p spec_index is out of range, or the
    ///        attribute's type or count differs.
    OUSTER_API_FUNCTION void check_attribute(std::size_t spec_index, AttributeType type,
                                             std::size_t count) const;

    /// @brief Write attribute @p spec_index of every record from typed values.
    ///
    /// Point `i`'s `count` values are read from `values[i * count]` onward,
    /// so a scalar attribute is one value per point and e.g. NORMALS
    /// `{F32, 3}` is three consecutive floats per point. The other attributes'
    /// bytes in @p records are left untouched.
    /// @tparam T The C++ type the attribute is stored as -- exactly, e.g.
    ///           `std::uint32_t` for U32; a same-sized `float` is rejected.
    ///           One of the eleven types AttributeType lists; any other type
    ///           fails to compile.
    /// @param[in]     spec_index  Which attribute to write.
    /// @param[in]     values      `value_count` values.
    /// @param[in]     value_count Must equal `records.rows() * count`.
    /// @param[in,out] records     `N` x @ref stride packed records.
    /// @throw std::invalid_argument if @p spec_index is out of range, @p T is
    ///        not the stored type, @p value_count is wrong, or @p records is
    ///        not @ref stride bytes wide.
    template <typename T>
    void pack(std::size_t spec_index, const T* values, std::size_t value_count,
              Eigen::Ref<AttributeRecords> records) const {
        pack_values(spec_index, impl::AttributeTypeOf<T>::value, values, value_count, records);
    }

    /// @brief `pack()` from a vector of `records.rows() * count` values.
    /// @param[in]     spec_index Which attribute to write.
    /// @param[in]     values     `records.rows() * count` values.
    /// @param[in,out] records    `N` x @ref stride packed records.
    /// @throw std::invalid_argument as the pointer overload.
    template <typename T>
    void pack(std::size_t spec_index, const std::vector<T>& values,
              Eigen::Ref<AttributeRecords> records) const {
        pack(spec_index, values.data(), values.size(), records);
    }

    /// @brief Read attribute @p spec_index of every record into typed values;
    /// the inverse of `pack()`, with the same value order.
    /// @tparam T The C++ type the attribute is stored as; see `pack()`.
    /// @param[in]  spec_index  Which attribute to read.
    /// @param[in]  records     `N` x @ref stride packed records.
    /// @param[out] values      `value_count` writable values.
    /// @param[in]  value_count Must equal `records.rows() * count`.
    /// @throw std::invalid_argument as `pack()`.
    template <typename T>
    void unpack(std::size_t spec_index, const Eigen::Ref<const AttributeRecords>& records,
                T* values, std::size_t value_count) const {
        unpack_values(spec_index, impl::AttributeTypeOf<T>::value, records, values, value_count);
    }

    /// @brief `unpack()` into a new vector.
    /// @param[in] spec_index Which attribute to read.
    /// @param[in] records    `N` x @ref stride packed records.
    /// @return `records.rows() * count` values.
    /// @throw std::invalid_argument as `pack()`.
    template <typename T>
    std::vector<T> unpack(std::size_t spec_index,
                          const Eigen::Ref<const AttributeRecords>& records) const {
        check_type(spec_index, impl::AttributeTypeOf<T>::value);
        std::vector<T> values(static_cast<std::size_t>(records.rows()) * specs_[spec_index].count);
        unpack(spec_index, records, values.data(), values.size());
        return values;
    }

    /// @brief Equality: same attributes in the same order.
    /// @param[in] other Layout to compare against.
    /// @return True when the layouts are identical.
    OUSTER_API_FUNCTION bool operator==(const AttributeLayout& other) const;

    /// @brief Inequality; see @ref operator==.
    /// @param[in] other Layout to compare against.
    /// @return True when the layouts differ.
    OUSTER_API_FUNCTION bool operator!=(const AttributeLayout& other) const {
        return !(*this == other);
    }

   private:
    // Throws std::out_of_range unless spec_index indexes specs_.
    void check_spec_index(std::size_t spec_index) const {
        if (spec_index >= specs_.size()) {
            throw std::out_of_range("spec_index " + std::to_string(spec_index) +
                                    " out of range for a layout of " +
                                    std::to_string(specs_.size()) + " attribute(s)");
        }
    }

    // Throws unless spec_index is in range and stored as `type`.
    OUSTER_API_FUNCTION void check_type(std::size_t spec_index, AttributeType type) const;

    // The type-erased bodies of pack() and unpack(): `values` points at
    // `value_count` scalars of `type`.
    OUSTER_API_FUNCTION void pack_values(std::size_t spec_index, AttributeType type,
                                         const void* values, std::size_t value_count,
                                         Eigen::Ref<AttributeRecords> records) const;
    OUSTER_API_FUNCTION void unpack_values(std::size_t spec_index, AttributeType type,
                                           const Eigen::Ref<const AttributeRecords>& records,
                                           void* values, std::size_t value_count) const;

    std::vector<AttributeSpec> specs_;
    std::vector<std::size_t> byte_offsets_;
    std::vector<std::size_t> element_offsets_;
    std::size_t stride_{0};
    std::size_t element_count_{0};
    bool all_double_{true};
};

/// @brief A single packed attribute record of `AttributeLayout::stride()` bytes.
using AttributeRecord = Eigen::VectorX<std::uint8_t>;

namespace impl {
/// Scalar type and element count of a struct member read by
/// AttributedPointCloud::attributes_as(): a scalar is one element, a C array
/// or std::array of scalars is one element per entry.
template <typename Field>
struct AttributeField {
    using Scalar = Field;
    static constexpr std::size_t count = 1;
};
template <typename Scalar_, std::size_t N>
struct AttributeField<Scalar_[N]> {
    using Scalar = Scalar_;
    static constexpr std::size_t count = N;
};
template <typename Scalar_, std::size_t N>
struct AttributeField<std::array<Scalar_, N>> {
    using Scalar = Scalar_;
    static constexpr std::size_t count = N;
};
}  // namespace impl

/// @brief A pointcloud whose attributes keep their original types.
///
/// Returned by @ref ouster::sdk::core::VoxelHashMap::pointcloud. The
/// two members are parallel: row `i` of @ref xyz is positioned by row `i` of
/// @ref attributes, whose bytes @ref layout describes.
///
/// The accessors below decode @ref attributes into typed C++ values, checking
/// the requested types against @ref layout at runtime; they never convert a
/// value, only copy its bytes, so what comes out is bit-for-bit what went in.
struct OUSTER_API_CLASS AttributedPointCloud {
    AttributeLayout layout;       ///< Layout of each row of @ref attributes.
    ArrayX3dR xyz;                ///< N x 3 point positions.
    AttributeRecords attributes;  ///< N x `layout.stride()` packed attribute records.

    /// @brief Number of points.
    /// @return The number of rows of @ref xyz and @ref attributes.
    OUSTER_API_FUNCTION std::size_t size() const;

    /// @brief Copy attribute @p spec_index of every point into @p out; see
    /// AttributeLayout::unpack().
    ///
    /// @tparam T The C++ type the attribute is stored as, e.g. `std::uint32_t`
    ///           for an AttributeType::U32 attribute. It must match exactly:
    ///           a same-sized but different type (`float` or `std::int32_t`
    ///           for a U32) is rejected rather than reinterpreted.
    /// @param[in]  spec_index Which attribute of @ref layout to read.
    /// @param[out] out        `out_count` writable values, filled point by
    ///                        point: point `i`'s values start at `out[i * count]`.
    /// @param[in]  out_count  Must equal `size() * count`.
    /// @throw std::invalid_argument if @p spec_index is out of range, @p T is
    ///        not the attribute's stored type, @p out_count is wrong, or
    ///        @ref xyz and @ref attributes disagree in shape.
    template <typename T>
    void attribute(std::size_t spec_index, T* out, std::size_t out_count) const {
        check_shape();
        layout.unpack(spec_index, attributes, out, out_count);
    }

    /// @brief Attribute @p spec_index of every point, in its stored type.
    ///
    /// For a scalar attribute (`count == 1`) element `i` belongs to point `i`;
    /// for a multi-element one, e.g. NORMALS `{F32, 3}`, point `i`'s values are
    /// elements `[i * count, (i + 1) * count)`.
    /// @tparam T The C++ type the attribute is stored as; see the overload
    ///           above.
    /// @param[in] spec_index Which attribute of @ref layout to read.
    /// @return `size() * count` values.
    /// @throw std::invalid_argument if @p spec_index is out of range, @p T is
    ///        not the attribute's stored type, or @ref xyz and @ref attributes
    ///        disagree in shape.
    template <typename T>
    std::vector<T> attribute(std::size_t spec_index) const {
        check_shape();
        return layout.unpack<T>(spec_index, attributes);
    }

    /// @brief Every point's attributes as a user-defined struct, verified
    /// field by field against @ref layout.
    ///
    /// @p fields lists the members of @p AttributesT that hold the layout's
    /// attributes, in layout order. Each member must have the attribute's
    /// stored type: a scalar for `count == 1`, or a C array / std::array of
    /// `count` scalars otherwise. Because every value is copied into its own
    /// member, @p AttributesT may have any padding or extra members:
    ///
    /// @code
    /// struct Attrs { std::uint32_t range; std::uint8_t reflectivity; };
    /// auto attrs = cloud.attributes_as<Attrs>(&Attrs::range, &Attrs::reflectivity);
    /// @endcode
    ///
    /// @tparam AttributesT Default-constructible struct to decode into.
    /// @param[in] fields   One pointer-to-member per attribute of @ref layout.
    /// @return One @p AttributesT per point.
    /// @throw std::invalid_argument if the number of fields, or any field's
    ///        type or element count, does not match @ref layout, or @ref xyz
    ///        and @ref attributes disagree in shape.
    template <typename AttributesT, typename... Fields>
    std::vector<AttributesT> attributes_as(Fields AttributesT::*... fields) const {
        static_assert(sizeof...(Fields) > 0, "list one member per attribute of the layout");
        check_shape();
        if (sizeof...(Fields) != layout.specs().size()) {
            throw std::invalid_argument("attributes_as: " + std::to_string(sizeof...(Fields)) +
                                        " field(s) given for a layout of " +
                                        std::to_string(layout.specs().size()) + " attribute(s)");
        }
        std::size_t index = 0;
        // C++14 stand-in for a fold expression: check every field in order.
        const int checks[] = {
            (layout.check_attribute(
                 index++,
                 impl::AttributeTypeOf<typename impl::AttributeField<Fields>::Scalar>::value,
                 impl::AttributeField<Fields>::count),
             0)...};
        (void)checks;

        // Once checked, each field's size is fixed at compile time; only the
        // offsets need looking up, once rather than per row.
        std::size_t offsets[sizeof...(Fields)];
        for (std::size_t spec = 0; spec < sizeof...(Fields); ++spec) {
            offsets[spec] = layout.byte_offset(spec);
        }

        std::vector<AttributesT> out(size());
        for (std::size_t row = 0; row < out.size(); ++row) {
            const std::uint8_t* record = attributes.row(static_cast<Eigen::Index>(row)).data();
            const std::size_t* offset = offsets;
            const int copies[] = {
                (std::memcpy(&(out[row].*fields), record + *offset++,
                             impl::AttributeField<Fields>::count *
                                 sizeof(typename impl::AttributeField<Fields>::Scalar)),
                 0)...};
            (void)copies;
        }
        return out;
    }

    /// @brief Every point's attributes as a packed struct mirroring the record.
    ///
    /// Copies each record straight into an @p AttributesT, so the struct must
    /// reproduce the record byte for byte: the layout's attributes in order,
    /// with no padding (records are packed, so `{U32, U8}` is 5 bytes, while a
    /// plain `struct { std::uint32_t; std::uint8_t; }` is usually 8). Declare
    /// it packed, e.g. with `#pragma pack(push, 1)`. Only the size can be
    /// checked here; prefer the field-listing overload, which also checks
    /// every member's type and needs no packing.
    ///
    /// @tparam AttributesT Trivially copyable struct of `layout.stride()` bytes.
    /// @return One @p AttributesT per point.
    /// @throw std::invalid_argument if `sizeof(AttributesT) != layout.stride()`,
    ///        or @ref xyz and @ref attributes disagree in shape.
    template <typename AttributesT>
    std::vector<AttributesT> attributes_as() const {
        static_assert(std::is_trivially_copyable<AttributesT>::value,
                      "AttributesT must be trivially copyable");
        std::vector<AttributesT> out(size());
        copy_records(out.data(), sizeof(AttributesT));
        return out;
    }

   private:
    // Throws unless there is one record per position and every record is
    // layout.stride() bytes wide, so the accessors never read or write past
    // the end of a buffer when the public members disagree.
    OUSTER_API_FUNCTION void check_shape() const;

    // Copies every record into `out`, an array of size() structs of
    // `struct_size` bytes each; throws unless struct_size == layout.stride().
    OUSTER_API_FUNCTION void copy_records(void* out, std::size_t struct_size) const;
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
