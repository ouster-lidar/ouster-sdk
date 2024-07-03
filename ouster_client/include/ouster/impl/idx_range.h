/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "ouster/impl/cuda_macros.h"

namespace ouster {
namespace impl {

struct idx_range {
    int start = 0;
    int end = 0;
};

template <typename T>
struct is_idx_range_t {
    static const int v = 0;
};

template <>
struct is_idx_range_t<idx_range> {
    static const int v = 1;
};

template <typename T, typename... Args>
struct count_n_ranges {
    static constexpr int value =
        count_n_ranges<T>::value + count_n_ranges<Args...>::value;
};

template <typename T>
struct count_n_ranges<T> {
    static constexpr int value = is_idx_range_t<std::decay_t<T>>::v;
};

template <typename T>
OSDK_FN constexpr int is_idx_range(T) {
    return is_idx_range_t<T>::v;
}

template <typename T>
OSDK_FN int range_or_idx(T idx) {
    return idx;
}

inline OSDK_FN int range_or_idx(idx_range e) { return e.start; }

template <typename T, typename U>
OSDK_FN std::enable_if_t<!is_idx_range_t<U>::v, T> range_replace_dim(T dim, U) {
    return dim;
}

template <typename T>
OSDK_FN T range_replace_dim(T dim, idx_range e) {
    if (e.end == 0) return dim;  // default empty idx_range

#ifndef __CUDA_ARCH__
    if (e.end <= e.start || e.end > static_cast<int>(dim)) {
        throw std::runtime_error("misshaped idx_range reshape");
    }
#endif

    return e.end - e.start;
}

/**
 * Shuffles arguments whether arg is an idx_range or not:
 *    skip array for args that are values, but retain for ranges
 */
template <int N, int M, typename... Args>
OSDK_FN void range_args_restride(const int32_t (&strides)[N],
                                 int32_t (&new_strides)[M], Args...) {
    static_assert(M == N - sizeof...(Args) + count_n_ranges<Args...>::value,
                  "shape mismatch");
    const bool skip[N] = {!static_cast<bool>(is_idx_range_t<Args>::v)...};

    int j = 0;
    for (int i = 0; i < N; ++i) {
        if (!skip[i]) new_strides[j++] = strides[i];
    }
}

/**
 * Same as above, but reworks shapes for shaped ranges
 */
template <int N, int M, typename... Args>
OSDK_FN void range_args_reshape(const int32_t (&shape)[N],
                                int32_t (&new_shape)[M], Args... args) {
    int i = 0;
    int32_t ranged_shape[N] = {range_replace_dim(shape[i++], args)...};
    for (; i < N; ++i) {
        ranged_shape[i] = shape[i];
    }

    range_args_restride(ranged_shape, new_shape, args...);
}

/**
 * Shuffles arguments whether arg is an idx_range or not:
 *    skip array for args that are values, but retain for ranges
 *
 * std::vector version
 */
template <typename T, typename... Args>
OSDK_FN_HOST std::vector<T> range_args_restride(const std::vector<T>& strides,
                                                Args...) {
    std::vector<T> out;

    int i = 0;

    for (bool skip : {!is_idx_range_t<Args>::v...}) {
        if (!skip) out.push_back(strides[i]);
        ++i;
    }

    for (int end = strides.size(); i < end; ++i) {
        out.push_back(strides[i]);
    }

    return out;
}

/**
 * Same as above, but reworks shapes for shaped ranges
 *
 * std::vector version
 */
template <typename T, typename... Args>
OSDK_FN_HOST std::vector<T> range_args_reshape(const std::vector<T>& shape,
                                               Args... args) {
    int i = 0;

    std::vector<T> ranged_shape = {range_replace_dim(shape[i++], args)...};
    for (int end = shape.size(); i < end; ++i) {
        ranged_shape.push_back(shape[i]);
    }

    return range_args_restride(ranged_shape, args...);
}

}  // namespace impl

OSDK_FN
inline impl::idx_range keep() { return {0, 0}; }
OSDK_FN
inline impl::idx_range keep(int start, int end) { return {start, end}; }

}  // namespace ouster
