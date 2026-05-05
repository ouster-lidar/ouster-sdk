#pragma once
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <functional>
#include <map>

#include "nonstd/optional.hpp"
#include "ouster/open_source.h"

// Common make opaques for each submodule. We need to include this in each.
PYBIND11_MAKE_OPAQUE(std::map<uint64_t, uint64_t>);

template <class IterT>
struct iterator_holder {
    IterT iter, end;
    bool first_or_done = true;
};

namespace py = pybind11;

namespace pybind11 {
namespace detail {
template <>
struct npy_format_descriptor<ouster::sdk::core::float16_t> {
    static pybind11::dtype dtype() {
        // 23 == NPY_HALF, from the NPY_TYPES enum in numpy/ndarraytypes.h
        // See: https://github.com/pybind/pybind11/issues/1776
        handle ptr = npy_api::get().PyArray_DescrFromType_(23);
        return reinterpret_borrow<pybind11::dtype>(ptr);
    }
    static std::string format() {
        // following:
        // https://docs.python.org/3/library/struct.html#format-characters "e"
        // is the struct format character for IEEE 754 binary16 (float16)
        return "e";
    }
    static constexpr auto name = const_name("float16");
};
}  // namespace detail
}  // namespace pybind11

void parse_packet_source_options(const pybind11::kwargs& args,
                                 ouster::sdk::PacketSourceOptions& options);
void parse_scan_source_options(const pybind11::kwargs& args,
                               ouster::sdk::ScanSourceOptions& options);

namespace pybind11 {
namespace detail {
template <typename T>
struct type_caster<nonstd::optional<T>> : optional_caster<nonstd::optional<T>> {
};
}  // namespace detail
}  // namespace pybind11
