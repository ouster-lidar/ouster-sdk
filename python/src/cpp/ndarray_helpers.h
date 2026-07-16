#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <stdexcept>
#include <type_traits>

#if WIN32
#include <BaseTsd.h>
using ssize_t = SSIZE_T;
#endif

namespace ouster {
namespace sdk {
namespace python {

template <typename>
inline constexpr bool is_ndarray_shape_v = false;

template <ssize_t... Is>
inline constexpr bool is_ndarray_shape_v<nanobind::shape<Is...>> = true;

template <typename Shape>
using shaped_ro_array = nanobind::ndarray<nanobind::ro, Shape>;

template <typename T, typename Shape>
using shaped_c_contig_array = nanobind::ndarray<const T, nanobind::c_contig, Shape>;

namespace detail {

inline bool is_floating_dtype(const nanobind::dlpack::dtype& dt) {
    return dt.code == static_cast<uint8_t>(nanobind::dlpack::dtype_code::Float) ||
           dt.code == static_cast<uint8_t>(nanobind::dlpack::dtype_code::Bfloat);
}

template <typename T, typename Array>
inline bool dtype_matches(const Array& arr) {
    return arr.dtype() == nanobind::dtype<T>();
}

template <typename Dst, typename Src>
inline Dst cast_ndarray(Src& arr) {
    return nanobind::cast<Dst>(arr.cast());
}

template <typename T>
inline nanobind::ndarray<const T, nanobind::c_contig> as_c_contig_typed(
    nanobind::ndarray<nanobind::ro>& arr) {
    if (!dtype_matches<T>(arr)) {
        throw nanobind::type_error("unexpected array dtype");
    }
    return cast_ndarray<nanobind::ndarray<const T, nanobind::c_contig>>(arr);
}

template <typename T, typename Shape>
inline shaped_c_contig_array<T, Shape> as_c_contig_typed(shaped_ro_array<Shape>& arr) {
    static_assert(is_ndarray_shape_v<Shape>, "Shape must be a nanobind::shape type");
    if (!dtype_matches<T>(arr)) {
        throw nanobind::type_error("unexpected array dtype");
    }
    return cast_ndarray<shaped_c_contig_array<T, Shape>>(arr);
}

template <typename T>
inline nanobind::ndarray<const T, nanobind::c_contig> as_c_contig_floating(
    nanobind::ndarray<nanobind::ro>& arr) {
    if (!is_floating_dtype(arr.dtype())) {
        throw nanobind::type_error("expected floating-point array");
    }
    return cast_ndarray<nanobind::ndarray<const T, nanobind::c_contig>>(arr);
}

template <typename T, typename Shape>
inline shaped_c_contig_array<T, Shape> as_c_contig_floating(shaped_ro_array<Shape>& arr) {
    static_assert(is_ndarray_shape_v<Shape>, "Shape must be a nanobind::shape type");
    if (!is_floating_dtype(arr.dtype())) {
        throw nanobind::type_error("expected floating-point array");
    }
    return cast_ndarray<shaped_c_contig_array<T, Shape>>(arr);
}

}  // namespace detail

template <typename T>
inline nanobind::ndarray<const T, nanobind::c_contig> ensure_c_contig(
    const nanobind::ndarray<nanobind::ro>& arr) {
    return detail::as_c_contig_typed<T>(const_cast<nanobind::ndarray<nanobind::ro>&>(arr));
}

template <typename T, typename Shape>
inline shaped_c_contig_array<T, Shape> ensure_c_contig(const shaped_ro_array<Shape>& arr) {
    return detail::as_c_contig_typed<T, Shape>(const_cast<shaped_ro_array<Shape>&>(arr));
}

template <typename T>
inline nanobind::ndarray<const T, nanobind::c_contig> ensure_c_contig_floating(
    const nanobind::ndarray<nanobind::ro>& arr) {
    return detail::as_c_contig_floating<T>(const_cast<nanobind::ndarray<nanobind::ro>&>(arr));
}

template <typename T, typename Shape>
inline shaped_c_contig_array<T, Shape> ensure_c_contig_floating(const shaped_ro_array<Shape>& arr) {
    return detail::as_c_contig_floating<T, Shape>(const_cast<shaped_ro_array<Shape>&>(arr));
}

}  // namespace python
}  // namespace sdk
}  // namespace ouster
