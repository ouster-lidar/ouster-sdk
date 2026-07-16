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

        // 1. Extract dimensions
        auto og_shape = shape;
        size_t str_len = shape.back();
        shape.pop_back();  // Remove the stride/string-length dim from logical
                           // shape

        size_t size = str_len;
        for (auto it : shape) {
            size *= it;
        }

        // 2. Create Raw MemoryView
        PyObject* raw_memview = PyMemoryView_FromMemory(
            reinterpret_cast<char*>(field.get()),
            static_cast<Py_ssize_t>(size * field.desc().element_size), PyBUF_WRITE);
        if (raw_memview == nullptr) {
            throw std::runtime_error("Failed to create memoryview");
        }
        py::object memv = py::steal<py::object>(raw_memview);

        // 3. Determine the Dtype String first
        py::object dtype_str;
        if (type.is_string()) {
            dtype_str = py::cast("S" + std::to_string(str_len));
            og_shape.pop_back();
        } else {
            dtype_str = type.otype();
        }

        // 4. Pass dtype to frombuffer
        // This creates an array of the correct logical size immediately.
        py::object array = numpy.attr("frombuffer")(memv, dtype_str);

        // 5. Reshape using a Tuple
        // Now valid because array.size matches product(shape)
        array = array.attr("reshape")(py::tuple(py::cast(og_shape)));

        // apply recarray
        // The python layer (client_common.h / dtype handling) likely maps this
        // to 'V1' This line casts the byte view to a structured array
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
