#include "client_common.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>  // For dlpack structs

#include <stdexcept>

#include "common.h"

// Include Python C-API for creating a raw memoryview
#include <Python.h>

const uint8_t* getptr(size_t bound, const py::object& buf) {
    // Attempt to view the object as a 1D uint8_t array (contiguous)
    // This handles bytes, bytearray, and other buffer-protocol objects
    py::ndarray<const uint8_t, py::ndim<1>, py::c_contig> arr;

    try {
        arr = py::cast<py::ndarray<const uint8_t, py::ndim<1>, py::c_contig>>(buf);
    } catch (const py::cast_error&) {
        throw std::invalid_argument(
            "Incompatible argument: expected a contiguous bytearray/buffer");
    }

    if (arr.size() < bound) {
        throw std::invalid_argument("Incompatible argument: expected a bytearray of size >= " +
                                    std::to_string(bound));
    }
    return arr.data();
}

py::object field_to_pyobj(ouster::sdk::core::FieldView& field, py::handle handle) {
    std::vector<size_t> shape = field.shape();
    if (shape.empty()) {
        throw std::invalid_argument("Field is not an array");
    }

    auto type = OusterDtype(field.tag());

    if (type.is_string() || type.is_object()) {
        py::module_ numpy = py::module_::import_("numpy");

        // Get type string
        py::object dtype_str;
        if (type.is_string()) {
            size_t str_len = shape.back();
            dtype_str = py::cast("S" + std::to_string(str_len));
            shape.pop_back();
            shape.back() *= str_len;
        } else {
            dtype_str = type.otype();
            shape.back() *= field.desc().element_size;
        }

        // Turn it into a byte array then view it on the python side as the correct dtype
        py::ndarray<py::numpy> out(field.get(), shape.size(), shape.data(), handle,
                                   nullptr,  // Contiguous
                                   py::dtype<uint8_t>());

        py::object array = out.cast();
        array = array.attr("view")(dtype_str);
        // turn it into a structured array
        array = array.attr("view")(numpy.attr("recarray"));
        return array;
    }

    // type.dtype() returns the py::dlpack::dtype struct
    // which the ndarray constructor accepts natively.
    py::ndarray<py::numpy> out(field.get(), shape.size(), shape.data(), handle,
                               nullptr,  // Contiguous
                               type.dtype());
    return out.cast();
}

ouster::sdk::core::FieldDescriptor pyarray_to_descriptor(const py::object& data) {
    // Ensure it's an array to access shape/dtype
    py::ndarray<py::numpy> arr;
    try {
        arr = py::cast<py::ndarray<py::numpy>>(data);
    } catch (const py::cast_error&) {
        throw std::invalid_argument("Input must be a numpy array");
    }

    // Initialize OusterDtype from the array's dtype
    auto type = OusterDtype(data.attr("dtype"));

    // Extract generic shape
    std::vector<size_t> shape;
    shape.reserve(arr.ndim());
    for (size_t i = 0; i < arr.ndim(); ++i) {
        shape.push_back(arr.shape(i));
    }

    if (type.is_string()) {
        size_t itemsize = py::cast<size_t>(data.attr("itemsize"));
        shape.push_back(itemsize);

        return ouster::sdk::core::FieldDescriptor::array(ouster::sdk::core::ChanFieldType::CHAR,
                                                         shape);
    } else if (type.is_object()) {
        // Handle Zone State or generic objects also works
        return ouster::sdk::core::FieldDescriptor::array(type.cft(), shape);
    } else {
        // Numeric types
        return ouster::sdk::core::FieldDescriptor::array(type.cft(), shape);
    }
}

ouster::sdk::core::FieldType init_field_type(const std::string& name, const py::object& dtype,
                                             std::vector<size_t> extra_dims,
                                             ouster::sdk::core::FieldClass field_class) {
    // Initialize helper with the passed numpy dtype object
    OusterDtype type(dtype);

    ouster::sdk::core::ChanFieldType cft = ouster::sdk::core::ChanFieldType::VOID;

    if (type.is_string()) {
        cft = ouster::sdk::core::ChanFieldType::CHAR;
        // For strings, the byte-width is stored in itemsize
        size_t itemsize = py::cast<size_t>(dtype.attr("itemsize"));
        if (itemsize > 0) {
            extra_dims.push_back(itemsize);
        }
    } else if (type.is_object()) {
        // Delegates to OusterDtype to determine if this is ZONE_STATE or
        // UNREGISTERED
        cft = type.cft();
    } else {
        // Numeric types
        cft = type.cft();
    }

    return ouster::sdk::core::FieldType(name, cft, extra_dims, field_class);
}
