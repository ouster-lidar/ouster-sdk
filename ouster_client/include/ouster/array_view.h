/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <exception>
#include <iterator>
#include <utility>

#include "ouster/impl/cuda_macros.h"
#include "ouster/impl/idx_range.h"

namespace ouster {

template <typename T, int Dim>
class ArrayView;

template <typename T, int Dim>
using ConstArrayView = ArrayView<const T, Dim>;

template <typename T>
using ArrayView4 = ArrayView<T, 4>;
template <typename T>
using ArrayView3 = ArrayView<T, 3>;
template <typename T>
using ArrayView2 = ArrayView<T, 2>;
template <typename T>
using ArrayView1 = ArrayView<T, 1>;

template <typename T>
using ConstArrayView4 = ConstArrayView<T, 4>;
template <typename T>
using ConstArrayView3 = ConstArrayView<T, 3>;
template <typename T>
using ConstArrayView2 = ConstArrayView<T, 2>;
template <typename T>
using ConstArrayView1 = ConstArrayView<T, 1>;

namespace impl {

template <int... I>
using row_pack = std::integer_sequence<int, I...>;

template <int N>
using index_row = std::make_integer_sequence<int, N>;

struct stop_recursion {};

template <int Left, int... I>
struct drop_left {
    using type = row_pack<I...>;
};

template <int Left>
struct drop_left<Left> {
    using type = stop_recursion;
};

template <typename T>
struct product {
    /**
     * Host-only version for iterators
     */
    template <typename IterT>
    T operator()(IterT, stop_recursion) {
        return T{1};
    }

    template <typename IterT, int... I>
    T operator()(IterT iter, row_pack<I...>) {
        IterT i = --iter;
        return static_cast<T>(*i) *
               (*this)(i, typename drop_left<I...>::type{});
    }

    /**
     * Device/host version for pointers
     */
    template <typename IntT>
    OSDK_FN T operator()(IntT*, stop_recursion) {
        return T{1};
    }

    template <typename IntT, int... I>
    OSDK_FN T operator()(IntT* ptr, row_pack<I...>) {
        IntT* i = --ptr;
        return static_cast<T>(*i) *
               (*this)(i, typename drop_left<I...>::type{});
    }

    /**
     * Device/host version for parameter pack
     */
    template <typename Arg>
    OSDK_FN T operator()(Arg arg) {
        return static_cast<T>(arg);
    }

    template <typename Arg, typename... Args>
    OSDK_FN T operator()(Arg arg, Args... args) {
        return (*this)(arg) * (*this)(args...);
    }
};

template <typename T>
struct restrict_if_const_ptr {
    using type = T*;
};

template <typename T>
struct restrict_if_const_ptr<const T> {
    using type = const T* RESTRICT;
};

// TODO: switch to fold expressions upon c++17 adoption
template <typename ShapeT, typename Arg>
bool _oob_check(const ShapeT& shape, int i, Arg idx0) {
    return impl::range_or_idx(idx0) >= static_cast<int>(shape[i]);
}

// TODO: switch to fold expressions upon c++17 adoption
template <typename ShapeT, typename Arg, typename... Args>
bool _oob_check(const ShapeT& shape, int i, Arg idx0, Args... idx) {
    return _oob_check(shape, i, idx0) + _oob_check(shape, i + 1, idx...);
}

// TODO: switch to fold expressions upon c++17 adoption
template <typename ShapeT, typename... Args>
bool subview_oob_check(const ShapeT& shape, Args... idx) {
    return _oob_check(shape, 0, idx...);
}

/**
 * TODO: this is a cpp14 regression, will not compile into CUDA.
 *       Fix later -- Tim T.
 */
// TODO: switch to fold expressions upon c++17 adoption
template <typename StridesT, typename Arg>
OSDK_FN int _strided_index(const StridesT& strides, int i, Arg idx0) {
    return idx0 * strides[i];
}

// TODO: switch to fold expressions upon c++17 adoption
template <typename StridesT, typename Arg, typename... Args>
OSDK_FN int _strided_index(const StridesT& strides, int i, Arg idx0,
                           Args... idx) {
    return _strided_index(strides, i, idx0) +
           _strided_index(strides, i + 1, idx...);
}

// TODO: switch to fold expressions upon c++17 adoption
template <typename StridesT, typename... Args>
OSDK_FN int strided_index(const StridesT& strides, Args... idx) {
    return _strided_index(strides, 0, idx...);
}

}  // namespace impl

template <typename T, int Dim>
class ArrayView {
    static_assert(Dim > 0, "ArrayView dimensions cannot be less than 1");

    // order of member declarations is important, most used variables first
    typename impl::restrict_if_const_ptr<T>::type data_;

   public:
    /**
     * C array of stride offsets
     */
    const int32_t strides[Dim];

    /**
     * C array of shape dimensions
     */
    const int32_t shape[Dim];

    /**
     * Construct from any types with subscript operator, i.e. vector/array
     *
     * host only due to cuda/stl incompatibility
     *
     * @param[in] ptr pointer to data
     * @param[in] shape arbitrary container of shape dimensions
     * @param[in] strides arbitrary container of stride offsets
     */
    template <typename Indexable1, typename Indexable2>
    ArrayView(T* ptr, const Indexable1& shape, const Indexable2& strides)
        : ArrayView(ptr, shape, strides, impl::index_row<Dim>{}) {}

    /**
     * Construct from any container type, deducing strides
     *
     * host only due to cuda/stl incompatibility
     *
     * @param[in] ptr pointer to data
     * @param[in] shape arbitrary container of shape dimensions
     */
    template <typename ContainerT>
    ArrayView(T* ptr, const ContainerT& shape)
        : ArrayView(ptr, std::begin(shape), std::end(shape),
                    impl::index_row<Dim>{}, impl::index_row<Dim - 1>{}) {}

    /**
     * Construct from C arrays
     *
     * device, host
     *
     * @param[in] ptr pointer to data
     * @param[in] shape C array of shape dimensions
     * @param[in] strides C array of stride offsets
     */
    OSDK_FN
    ArrayView(T* ptr, const int (&shape)[Dim], const int (&strides)[Dim])
        : ArrayView(ptr, shape, strides, impl::index_row<Dim>{}) {}

    /**
     * Construct from initializer list, deducing strides
     *
     * device, host
     *
     * @param[in] ptr pointer to data
     * @param[in] shape initializer_list of shape dimensions
     */
    template <typename IntT>
    OSDK_FN ArrayView(T* ptr, std::initializer_list<IntT> shape)
        : ArrayView(ptr, shape.begin(), shape.end(), impl::index_row<Dim>{},
                    impl::index_row<Dim - 1>{}) {}

    /**
     * Construct from C array, deducing strides
     *
     * device, host
     *
     * @param[in] ptr pointer to data
     * @param[in] shape C array of shape dimensions
     */
    OSDK_FN
    ArrayView(T* ptr, const int (&shape)[Dim])
        : ArrayView(ptr, shape, shape + Dim, impl::index_row<Dim>{},
                    impl::index_row<Dim - 1>{}) {}

    /**
     * Below allows conversion from ArrayView to ConstArrayView
     * but not the other way around
     *
     * /code
     * ArrayView4<int> a;
     * ConstArrayView4<int> b = a;  // compiles
     * ArrayView4<int> c = b;       // does not compile
     * /endcode
     *
     * @param[in] other ArrayView
     */
    template <typename OtherT, typename = std::enable_if_t<
                                   std::is_same<T, const OtherT>::value, void>>
    OSDK_FN ArrayView(const ArrayView<OtherT, Dim>& other)
        : ArrayView(other.data_, other.shape, other.strides) {}

    /**
     * Retrieve reference to value with subscript operator
     *
     * @param[in] idx indices
     *
     * @return reference to value
     */
    template <typename... Args>
    OSDK_FN const T& operator()(Args&&... idx) const {
        static_assert(sizeof...(Args) == Dim,
                      "ArrayView subscript operator "
                      "must match the array "
                      "dimensions");

        return *(data_ + impl::strided_index(strides, idx...));
    }

    /**
     * Retrieve reference to value with subscript operator
     *
     * @param[in] idx indices
     *
     * @return reference to value
     */
    template <typename... Args>
    OSDK_FN T& operator()(Args&&... idx) {
        return const_cast<T&>(
            const_cast<const ArrayView&>(*this)(std::forward<Args>(idx)...));
    }

    /**
     * Templated utility to calculate subview dimensions at compile time
     *
     * @tparam Args parameter pack of indices or idx_range
     *
     * @return subview dimension
     */
    template <typename... Args>
    OSDK_FN static constexpr int subview_dim() {
        return Dim - sizeof...(Args) + impl::count_n_ranges<Args...>::value;
    }

    /**
     * Get a subview
     *
     * Operates similarly to numpy ndarray bracket operator, returning a sliced,
     * potentially sparse subview
     *
     * The following two snippets are functionally equivalent
     * \code
     * std::vector<int> data(100*100*100);
     * ArrayView3<int> view{data.data(), {100,100,100};
     * // get a slice of all elements in the first dimension with second
     * // and third pinned to 10 and 20 respectively
     * ArrayView1<int> subview = view.subview(keep(), 10, 20);
     * \endcode
     *
     * \code
     * import numpy as np
     * arr = np.ndarray(shape=(100,100,100), dtype=np.int32)
     * subview = arr[:,10,20]
     * \endcode
     *
     * @param[in] idx pack of int indices or idx_range (keep())
     *
     * @return subview with different dimensions
     */
    template <typename... Args>
    OSDK_FN ArrayView<T, subview_dim<Args...>()> subview(Args... idx) const {
        constexpr int N = subview_dim<Args...>();
        static_assert(N > 0, "Ran out of dimensions to subview");

#ifndef __CUDA_ARCH__
        if (impl::subview_oob_check(shape, idx...))
            throw std::invalid_argument(
                "ArrayView cannot subview over the "
                "shape limits");
#endif

        T* ptr =
            data_ + impl::strided_index(strides, impl::range_or_idx(idx)...);
        int32_t new_shape[N];
        int32_t new_strides[N];
        impl::range_args_reshape(shape, new_shape, idx...);
        impl::range_args_restride(strides, new_strides, idx...);
        return ArrayView<T, N>(ptr, new_shape, new_strides);
    }

    /**
     * Reshape array view to a different set of dimensions
     *
     * @throw std::invalid_argument on trying to reshape a sparse ArrayView
     * @throw std::invalid_argument on flattened dimension size not matching
     *        the original view size
     *
     * @param[in] dims new dimensions
     *
     * @return reshaped ArrayView with new dimensions
     */
    template <typename... Args>
    ArrayView<T, sizeof...(Args)> reshape(Args... dims) const {
        if (sparse()) {
            throw std::invalid_argument(
                "ArrayView: cannot reshape sparse views");
        }

        if (impl::product<int>{}(dims...) != shape[0] * strides[0]) {
            throw std::invalid_argument(
                "ArrayView: cannot reshape due to size mismatch");
        }

        return ArrayView<T, sizeof...(Args)>(data_,
                                             {static_cast<int>(dims)...});
    }

    /**
     * Check if the ArrayView is not contiguous
     *
     * @return true if sparse
     */
    OSDK_FN bool sparse() const {
        bool dense = strides[Dim - 1] == 1;
        for (int i = 1; i < Dim; ++i) {
            dense = dense && (strides[i - 1] == strides[i] * shape[i]);
        }
        return !dense;
    }

    /**
     * Iterator functions below return basic pointers to data, which only works
     * if the current array is not sparse.
     *
     * Because we have no way of throwing if the array is sparse, these are
     * explicitly host side at the moment, until we get to implementing
     * sparse iterators
     *
     * @return pointer to data
     */
    const T* begin() const {
        if (sparse())
            throw std::logic_error(
                "ArrayView iterators not supported for "
                "sparse views");
        return data_;
    }

    // @copydoc begin()
    const T* end() const {
        if (sparse())
            throw std::logic_error(
                "ArrayView iterators not supported "
                "for sparse views");
        return data_ + shape[0] * strides[0];
    }

    // @copydoc begin()
    T* begin() {
        return const_cast<T*>(const_cast<const ArrayView*>(this)->begin());
    }

    // @copydoc begin()
    T* end() {
        return const_cast<T*>(const_cast<const ArrayView*>(this)->end());
    }

    /**
     * Get the underlying data pointer.
     *
     * @return pointer to data
     */
    const T* data() const { return data_; }

    // @copydoc data()
    T* data() { return data_; }

   private:
    friend class ArrayView<const T, Dim>;

    /**
     * Internal indexable based constructor
     *
     * host only due to index subscript
     */
    template <typename Indexable1, typename Indexable2, int... I>
    ArrayView(T* ptr, const Indexable1& _shape, const Indexable2& _strides,
              impl::row_pack<I...>)
        : data_(ptr),
          strides{static_cast<int32_t>(_strides[I])...},
          shape{static_cast<int32_t>(_shape[I])...} {}

    /**
     * Internal iterator based constructor
     *
     * host only due to iterators
     */
    template <typename IterT, int... I, int... J>
    ArrayView(T* ptr, IterT first, IterT last, impl::row_pack<I...>,
              impl::row_pack<J...>)
        : data_(ptr),
          strides{
              impl::product<int32_t>{}(last, impl::index_row<Dim - 1 - J>{})...,
              1},
          shape{(void(I), static_cast<int32_t>(*first++))...} {}

    /**
     * Internal pointer pair based constructor
     */
    template <typename IntT, int... I, int... J>
    OSDK_FN ArrayView(T* ptr, IntT* first, IntT* last, impl::row_pack<I...>,
                      impl::row_pack<J...>)
        : data_(ptr),
          strides{
              impl::product<int32_t>{}(last, impl::index_row<Dim - 1 - J>{})...,
              1},
          shape{(void(I), static_cast<int32_t>(*first++))...} {
        (void)last;  // silence an unused variable warning due to a compiler bug
    }

    /**
     * Internal C array based constructor
     */
    template <int... I>
    OSDK_FN ArrayView(T* ptr, const int32_t (&shape)[Dim],
                      const int32_t (&strides)[Dim], impl::row_pack<I...>)
        : data_(ptr), strides{strides[I]...}, shape{shape[I]...} {}
};

}  // namespace ouster
