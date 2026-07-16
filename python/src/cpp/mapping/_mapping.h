#pragma once

// Shared helpers and forward declarations for the split _mapping translation
// units.  All mapping TUs include this header.

#define FMT_UNICODE 0

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <cstddef>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>

#include "common.h"
#include "eigen_dense.h"

// Convenience array types reused across pose-delta and optimizer TUs.
using Mat4Input = py::ndarray<py::numpy, const double, py::shape<4, 4>, py::c_contig>;
using ArrayX3Input = py::ndarray<py::numpy, const double, py::shape<-1, 3>, py::c_contig>;

// Heap-allocate a 1-D array of T and wrap it in a nanobind capsule so numpy
// owns the memory.
template <typename T>
inline py::capsule make_capsule(T* data) {
    return py::capsule(data, [](void* pointer) noexcept { delete[] static_cast<T*>(pointer); });
}

// Allocate and return a numpy array with the given shape.
template <typename T, int ndim = 1>
inline py::ndarray<py::numpy, T, py::ndim<ndim>, py::c_contig> make_array(
    const std::vector<size_t>& shape) {
    size_t size = 0;
    for (const auto& dim : shape) {
        size = (size == 0) ? dim : size * dim;
    }
    T* data = new T[size];
    py::ndarray<py::numpy, T, py::ndim<ndim>, py::c_contig> result(data, shape.size(), shape.data(),
                                                                   make_capsule(data));
    return result;
}

// Forward declarations for the sub-init functions called by init_mapping().
void init_mapping_config(py::module_& module);
void init_mapping_constraints(py::module_& module);
void init_mapping_pose_optimizer(py::module_& module);
void init_mapping_slam(py::module_& module);
void init_mapping_registration(py::module_& module);
