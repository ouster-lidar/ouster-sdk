/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <stdlib.h>  // for size_t since gcc-12

#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <type_traits>
#include <typeindex>
#include <unordered_map>
#include <vector>

#include "ouster/array_view.h"
#include "ouster/types.h"

namespace ouster {

namespace impl {

/**
 * Calculates vector of stride offsets
 *
 * @param[in] shape vector of array dimensions
 *
 * @return vector of stride offsets
 */
std::vector<size_t> calculate_strides(const std::vector<size_t>& shape);

}  // namespace impl

namespace sensor {
namespace impl {

// clang-format off

template <typename T> int type_size() { return sizeof(T); }
template <> int type_size<void>();

template <typename T> size_t type_hash() { return typeid(T).hash_code(); }
template <> size_t type_hash<void>();

template <typename T>
ChanFieldType type_cft() { return ChanFieldType::UNREGISTERED; }
template <> ChanFieldType type_cft<void>();
template <> ChanFieldType type_cft<uint8_t>();
template <> ChanFieldType type_cft<uint16_t>();
template <> ChanFieldType type_cft<uint32_t>();
template <> ChanFieldType type_cft<uint64_t>();
template <> ChanFieldType type_cft<int8_t>();
template <> ChanFieldType type_cft<int16_t>();
template <> ChanFieldType type_cft<int32_t>();
template <> ChanFieldType type_cft<int64_t>();
template <> ChanFieldType type_cft<float>();
template <> ChanFieldType type_cft<double>();

// clang-format on

}  // namespace impl
}  // namespace sensor

/**
 * Helper struct used by FieldView and Field to describe field contents.
 * Unlike FieldType this fully describes a Field's dimensions rather than
 * abstract away the lidar width and height or packet count.
 */
struct FieldDescriptor {
    /**
     * type hash of the described field
     */
    size_t type;

    /**
     * Calculates the size in bytes of the described field
     *
     * @return type size in bytes
     */
    size_t bytes() const;

    // TODO: ideally we need something like llvm::SmallVector here -- Tim T.

    /**
     * vector of array dimensions of the described field, if present
     */
    std::vector<size_t> shape;

    /**
     * vector of stride offsets of the described field, if present
     */
    std::vector<size_t> strides;

    /**
     * size of the underlying type, in bytes
     *
     */
    size_t element_size;

    /**
     * Get type hash
     *
     * warning: different platforms produce different values
     *
     * @return hash value
     */
    template <typename T>
    static size_t type_hash() {
        using Type = typename std::remove_cv<T>::type;
        return sensor::impl::type_hash<Type>();
    }

    /**
     * Get array size in elements, or 1 if shape is not present
     *
     * @return size in elements
     */
    size_t size() const;

    /**
     * Get type tag, if can be translated to ChanFieldType, otherwise
     * returns ChanFieldType::UNREGISTERED
     *
     * @return ChanFieldType
     */
    sensor::ChanFieldType tag() const;

    bool operator==(const FieldDescriptor& other) const noexcept {
        return type == other.type && shape == other.shape &&
               strides == other.strides && element_size == other.element_size;
    }

    /**
     * Swaps descriptors
     *
     * @param[in,out] other Handle to swapped FieldDescriptor.
     */
    void swap(FieldDescriptor& other);

    /**
     * Check if the type is eligible for conversion
     *
     * @return true if eligible, otherwise false.
     */
    template <typename T>
    bool eligible_type() const {
        // TODO: reinstate upon c++17 -- Tim T.
        if /*constexpr*/ (
            std::is_same<void, typename std::remove_cv<T>::type>::value) {
            return true;
        }
        return !type || type_hash<T>() == type;
    }

    /**
     * Check if descriptor types are compatible
     *
     * @param[in] other A constant of type FieldDescriptor.
     * @return true if compatible, otherwise false.
     */
    bool is_type_compatible(const FieldDescriptor& other) const noexcept;

    /**
     * Return number of dimensions of the described field
     *
     * @return number of dimensions.
     */
    size_t ndim() const noexcept;

    // Factory functions

    /**
     * Get a field descriptor for a chunk of typed memory
     *
     * useful when storing arbitrary sized structs.
     *
     * @tparam T Type of memory to be stored; this gets used by safety checks.
     * @param[in] bytes Number of bytes in memory.
     *
     * @return FieldDescriptor
     */
    template <typename T = void>
    static FieldDescriptor memory(size_t bytes) {
        return {type_hash<T>(), {}, {}, bytes};
    }

    /**
     * Get a field descriptor for an array
     *
     * @tparam T Array type
     * @param[in] shape Shape vector of array dimensions.
     *
     * @return FieldDescriptor
     */
    template <typename T, class ContainerT = std::initializer_list<size_t>>
    static FieldDescriptor array(const ContainerT& shape) {
        static_assert(!std::is_same<T, void>::value,
                      "FieldDescriptor::array<void> is disallowed, use "
                      "FieldDescriptor::memory() instead");
        return {type_hash<T>(), shape, impl::calculate_strides(shape),
                sizeof(T)};
    }

    /**
     * Get a field descriptor for an array
     *
     * @param[in] tag Tag of array type.
     * @param[in] shape Vector of array dimensions.
     *
     * @return FieldDescriptor
     */
    template <class ContainerT = std::initializer_list<size_t>>
    static FieldDescriptor array(sensor::ChanFieldType tag,
                                 const ContainerT& shape) {
        switch (tag) {
            case sensor::ChanFieldType::UINT8:
                return array<uint8_t>(shape);
            case sensor::ChanFieldType::UINT16:
                return array<uint16_t>(shape);
            case sensor::ChanFieldType::UINT32:
                return array<uint32_t>(shape);
            case sensor::ChanFieldType::UINT64:
                return array<uint64_t>(shape);
            case sensor::ChanFieldType::INT8:
                return array<int8_t>(shape);
            case sensor::ChanFieldType::INT16:
                return array<int16_t>(shape);
            case sensor::ChanFieldType::INT32:
                return array<int32_t>(shape);
            case sensor::ChanFieldType::INT64:
                return array<int64_t>(shape);
            case sensor::ChanFieldType::FLOAT32:
                return array<float>(shape);
            case sensor::ChanFieldType::FLOAT64:
                return array<double>(shape);
            default:
                throw std::invalid_argument(
                    "fd_array: unsupported ChanFieldType");
        }
    }
};

/**
 * Parameter pack shorthand for FieldDescriptor::array
 * @param[in] args Variadic arguments that are forwarded to the function.
 * @return FieldDescriptor array
 */
template <typename T, typename... Args>
auto fd_array(Args&&... args) -> FieldDescriptor {
    return FieldDescriptor::array<T>({static_cast<size_t>(args)...});
}

// @copydoc fd_array()
template <typename... Args>
auto fd_array(sensor::ChanFieldType tag, Args&&... args) -> FieldDescriptor {
    return FieldDescriptor::array(tag, {static_cast<size_t>(args)...});
}

/**
 * Non-owning wrapper over a memory pointer that allows for type safe
 * conversion to typed pointer, eigen array or ArrayView
 */
class FieldView {
   protected:
    void* ptr_;
    FieldDescriptor desc_;

   public:
    /** Default constructor for empty FieldView */
    FieldView() noexcept;

    /**
     * Initialize FieldView with a pointer and a descriptor
     *
     * @param[in] ptr Memory pointer.
     * @param[in] desc Field descriptor.
     */
    FieldView(void* ptr, const FieldDescriptor& desc);

    /**
     * Returns arbitrary pointer type
     *
     * @throw std::invalid_argument on type mismatch unless FieldView was
     * constructed typeless (with type void) or dereference type is void*
     *
     * @tparam T pointer type
     * @return typed pointer to the memory
     */
    template <typename T = void>
    T* get() {
        return const_cast<T*>(const_cast<const FieldView&>(*this).get<T>());
    }

    // @copydoc get()
    template <typename T = void>
    const T* get() const {
        if (!desc_.eligible_type<T>()) {
            throw std::invalid_argument(
                "FieldView: ineligible dereference type");
        }
        return reinterpret_cast<T*>(ptr_);
    }

    /**
     * Arbitrary type ptr conversion

     * @throw std::invalid_argument on type mismatch unless FieldView was
     * constructed with void ptr or the requested ptr type is void*
     */
    template <typename T>
    operator T*() {
        return get<T>();
    }

    /**
     * Arbitrary type const ptr conversion

     * @throw std::invalid_argument on type mismatch unless FieldView was
     * constructed with void ptr or the requested ptr type is void*
     */
    template <typename T>
    operator const T*() const {
        return get<T>();
    }

    /**
     * Arbitrary type ArrayView conversion

     * @throw std::invalid_argument on type mismatch
     * @throw std::invalid_argument on dimensional shape mismatch
     */
    template <typename T, int Dim>
    operator ArrayView<T, Dim>() {
        if (desc_.ndim() != Dim) {
            throw std::invalid_argument(
                "FieldView: ArrayView conversion failed due to dimension "
                "mismatch");
        }

        return ArrayView<T, Dim>(get<T>(), desc_.shape, desc_.strides);
    }

    /**
     * Arbitrary type const ArrayView conversion

     * @throw std::invalid_argument on type mismatch
     * @throw std::invalid_argument on dimensional shape mismatch
     */
    template <typename T, int Dim>
    operator ConstArrayView<T, Dim>() const {
        if (desc_.ndim() != Dim) {
            throw std::invalid_argument(
                "FieldView: ArrayView conversion failed due to dimension "
                "mismatch");
        }

        return ConstArrayView<T, Dim>(get<T>(), desc_.shape, desc_.strides);
    }

    /**
     * Arbitrary type Eigen 2D array conversion
     *
     * @throw std::invalid_argument on type mismatch
     * @throw std::invalid_argument on dimensional shape mismatch
     */
    template <typename T>
    operator Eigen::Ref<img_t<T>>() {
        if (desc_.ndim() != 2) {
            throw std::invalid_argument(
                "Field: Eigen array conversion failed due to dimension "
                "mismatch");
        }

        if (sparse()) {
            throw std::invalid_argument(
                "Field: Cannot convert sparse view to dense Eigen array");
        }

        Eigen::Index h = shape()[0], w = shape()[1];
        return Eigen::Map<img_t<T>>{get<T>(), h, w};
    }

    /**
     * Arbitrary type const Eigen 2D array conversion
     *
     * @throw std::invalid_argument on type mismatch
     * @throw std::invalid_argument on dimensional shape mismatch
     */
    template <typename T>
    operator Eigen::Ref<const img_t<T>>() const {
        if (desc_.ndim() != 2) {
            throw std::invalid_argument(
                "Field: Eigen array conversion failed due to dimension "
                "mismatch");
        }

        if (sparse()) {
            throw std::invalid_argument(
                "Field: Cannot convert sparse view to dense Eigen array");
        }

        Eigen::Index h = shape()[0], w = shape()[1];
        return Eigen::Map<const img_t<T>>{get<T>(), h, w};
    }

    /**
     * Arbitrary type const Eigen 1D array conversion
     *
     * @throw std::invalid_argument on type mismatch
     * @throw std::invalid_argument on dimensional shape mismatch
     */
    template <typename T>
    operator Eigen::Ref<Eigen::Array<T, Eigen::Dynamic, 1>>() {
        if (desc_.ndim() != 1) {
            throw std::invalid_argument(
                "Field: Eigen array conversion failed due to dimension "
                "mismatch");
        }

        if (sparse()) {
            throw std::invalid_argument(
                "Field: Cannot convert sparse view to dense Eigen array");
        }

        Eigen::Index w = shape()[0];
        return Eigen::Map<Eigen::Array<T, Eigen::Dynamic, 1>>{get<T>(), w};
    }

    /**
     * Arbitrary type const Eigen 1D array conversion
     *
     * @throw std::invalid_argument on type mismatch
     * @throw std::invalid_argument on dimensional shape mismatch
     */
    template <typename T>
    operator Eigen::Ref<const Eigen::Array<T, Eigen::Dynamic, 1>>() const {
        if (desc_.ndim() != 1) {
            throw std::invalid_argument(
                "Field: Eigen array conversion failed due to dimension "
                "mismatch");
        }

        if (sparse()) {
            throw std::invalid_argument(
                "Field: Cannot convert sparse view to dense Eigen array");
        }

        Eigen::Index w = shape()[0];
        return Eigen::Map<const Eigen::Array<T, Eigen::Dynamic, 1>>{get<T>(),
                                                                    w};
    }

    /**
     * Bool conversion
     *
     * @return true if FieldView is owning a resource
     */
    explicit operator bool() const noexcept;

    /**
     * Get a subview
     *
     * Operates similarly to numpy ndarray bracket operator, returning a sliced,
     * potentially sparse subview
     *
     * The following two snippets are functionally equivalent
     * \code
     * std::vector<int> data(100*100*100);
     * FieldView view(data.data(), fd_array<int>(100, 100, 100));
     * // get a slice of all elements in the first dimension with second
     * // and third pinned to 10 and 20 respectively
     * FieldView subview = view.subview(keep(), 10, 20);
     * \endcode
     *
     * \code
     * import numpy as np
     * arr = np.ndarray(shape=(100,100,100), dtype=np.int32)
     * subview = arr[:,10,20]
     * \endcode
     *
     * @throws std::invalid_argument If FieldView ran out of dimensions to
     *         subview or if FieldView cannot subview over the shape limits
     * @param[in] idx parameter pack of int indices or idx_range (keep())
     *
     * @return FieldView subview
     */
    template <typename... Args>
    FieldView subview(Args... idx) const {
        auto dim = desc().ndim();
        auto subview_dim =
            dim - sizeof...(Args) + impl::count_n_ranges<Args...>::value;

        if (subview_dim <= 0)
            throw std::invalid_argument(
                "FieldView: ran out of dimensions to subview");

        if (impl::subview_oob_check(shape(), idx...)) {
            throw std::invalid_argument(
                "FieldView cannot subview over the shape limits");
        }

        char* ptr =
            reinterpret_cast<char*>(ptr_) +
            impl::strided_index(desc().strides, impl::range_or_idx(idx)...) *
                desc().element_size;

        auto new_desc = FieldDescriptor{};
        new_desc.type = desc().type;
        new_desc.shape = impl::range_args_reshape(desc().shape, idx...);
        new_desc.strides = impl::range_args_restride(desc().strides, idx...);
        new_desc.element_size = desc().element_size;

        return {reinterpret_cast<void*>(ptr), new_desc};
    }

    /**
     * Reshape field view to a different set of dimensions
     *
     * @throw std::invalid_argument on trying to reshape a sparse FieldView
     * @throw std::invalid_argument on flattened dimension size not matching
     *        the original view size
     *
     * @param[in] dims new dimensions
     *
     * @return reshaped FieldView with new dimensions
     */
    template <typename... Args>
    FieldView reshape(Args... dims) const {
        if (sparse()) {
            throw std::invalid_argument(
                "FieldView: cannot reshape sparse views");
        }

        if (impl::product<size_t>{}(dims...) != size()) {
            throw std::invalid_argument(
                "ArrayView: cannot reshape due to size mismatch");
        }

        auto new_desc = FieldDescriptor{};
        new_desc.type = desc().type;
        new_desc.element_size = desc().element_size;
        new_desc.shape = std::vector<size_t>{static_cast<size_t>(dims)...};
        new_desc.strides = impl::calculate_strides(new_desc.shape);
        return {const_cast<void*>(ptr_), new_desc};
    }

    /**
     * Returns the number of allocated bytes in memory
     *
     * @return size in bytes
     */
    size_t bytes() const noexcept;

    /**
     * Returns size in elements, or 1 if field is not an array
     *
     * @return size in elements
     */
    size_t size() const;

    /**
     * returns true if FieldView matches descriptor
     *
     * @param[in] d descriptor to check
     *
     * @return true if matched, otherwise false
     */
    bool matches(const FieldDescriptor& d) const noexcept;

    /**
     * Get descriptor of the underlying memory
     *
     * @return FieldDescriptor
     */
    const FieldDescriptor& desc() const;

    /**
     * Get shape of the stored array, if present
     *
     * shorthand for `desc().shape`
     *
     * @return vector of dimensions
     */
    const std::vector<size_t>& shape() const;

    /**
     * Get type tag, if applicable
     *
     * @return ChanFieldType
     */
    sensor::ChanFieldType tag() const;

    /**
     * Check if the FieldView is not contiguous
     *
     * @return true if sparse
     */
    bool sparse() const;
};

/**
 * Classes of LidarScan fields
 */
enum class FieldClass {
    /**
     * Corresponds to fields of (height, width, ...) dimensions
     */
    PIXEL_FIELD = 1,

    /**
     * Corresponds to fields that have first dimension equal to width
     */
    COLUMN_FIELD = 2,

    /**
     * Corresponds to fields that have first dimension equal to number
     * of packets necessary to construct a complete LidarScan
     */
    PACKET_FIELD = 3,

    /**
     * Corresponds to fields of any dimension associated with the scan
     * as a whole rather than any pixel, column or packet
     */
    SCAN_FIELD = 4,
};

/**
 * Get string representation of singular FieldClass flag
 *
 * @param[in] f flag to get the string representation for.
 *
 * @return string representation of the FieldClass, or "UNKNOWN".
 */
std::string to_string(FieldClass f);

/**
 * RAII memory-owning container for arbitrary typed and sized arrays and POD
 * structs with optional type checking
 *
 * For usage examples, check unit tests
 */
class Field : public FieldView {
   protected:
    FieldClass class_;

   public:
    /** Default constructor, representing invalid Field */
    Field() noexcept;

    /** Field destructor */
    ~Field();

    /**
     * Constructs Field from FieldDescriptor
     *
     * @param[in] desc FieldDescriptor
     * @param[in] field_class FieldClass
     */
    Field(const FieldDescriptor& desc, FieldClass field_class = {});

    /**
     * Copy constructor
     *
     * @param[in] other Field to copy
     */
    Field(const Field& other);

    /**
     * Copy assignment constructor
     *
     * @param[in] other Field to copy
     */
    Field& operator=(const Field& other);

    /**
     * Move constructor
     *
     * @param[in] other Field to steal resource from
     */
    Field(Field&& other) noexcept;

    /**
     * Move assignment constructor
     *
     * @param[in] other Field to steal resource from
     */
    Field& operator=(Field&& other) noexcept;

    /**
     * Get field class
     *
     * @return FieldClass
     */
    FieldClass field_class() const;

    /**
     * Swaps two Fields
     *
     * @param[in] other Field to swap resources with
     */
    void swap(Field& other) noexcept;

    /**
     * Equality operator
     *
     * @return true if type, shape and memory contents are equal to other
     */
    bool operator==(const Field& other) const;
};

/**
 * Acquire a uintXX_t reinterpreted view of the matching type size.
 * Useful for memory related operations like parsing and compression.
 *
 * WARNING: reinterprets the type skipping type safety checks, exercise caution
 *
 * @throw std::invalid_argument if `other` is not an array view
 * @throw std::invalid_argument if `other` is an array view of custom POD types
 *        that do not match any uintXX_t dimensions
 *
 * @param[in] other view to interpret
 *
 * @return reinterpreted view of uint8_t, uint16_t, uint32_t or uint64_t type
 */
FieldView uint_view(const FieldView& other);

}  // namespace ouster

namespace std {

/**
 * std::swap overload, used by some std algorithms
 *
 * @param[in] a Field to swap with b
 * @param[in] b Field to swap with a
 */
void swap(ouster::Field& a, ouster::Field& b);

}  // namespace std
