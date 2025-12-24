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
