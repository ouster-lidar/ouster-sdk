/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief ouster_pyclient
 *
 * Note: the type annotations in `client.pyi` need to be updated whenever this
 * file changes. See the mypy documentation for details.
 */

#pragma once
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "common.h"
#include "ouster/impl/packet_writer.h"
#include "ouster/lidar_scan.h"
#include "ouster/packet.h"
#include "ouster/typedefs.h"
#include "ouster/types.h"
#if WIN32
#include <BaseTsd.h>
using ssize_t = SSIZE_T;
#endif

/*
 * Check that buffer is a 1-d byte array of size > bound and return an
 * internal pointer to the data for writing. Check is strictly greater
 * to account for the extra byte required to determine if a datagram
 * is bigger than expected.
 */
inline uint8_t* getptr(size_t bound, pybind11::buffer& buf) {
    auto info = buf.request();
    if (info.format != pybind11::format_descriptor<uint8_t>::format() ||
        // type of info.size has changed from size_t to ssize_t
        info.ndim != 1 || info.size < static_cast<decltype(info.size)>(bound)) {
        throw std::invalid_argument(
            "Incompatible argument: expected a bytearray of size >= " +
            std::to_string(bound));
    }
    return static_cast<uint8_t*>(info.ptr);
}

template <typename T>
struct SetField {
    using Field = pybind11::array_t<T, pybind11::array::c_style |
                                           pybind11::array::forcecast>;

    void operator()(const ouster::sdk::core::impl::PacketWriter& self,
                    ouster::sdk::core::LidarPacket& packet,
                    const std::string& field_name_str, Field field) {
        if (field.ndim() != 2 || field.shape(0) != self.pixels_per_column ||
            field.shape(1) != self.columns_per_packet) {
            throw std::invalid_argument("field dimension mismatch");
        }

        /**
         * It is a bit weird to be setting these back and forth to work around
         * PacketWriter::set_block logic that is intended for lidarscan usage
         * but I do think it is better than keeping two versions of the same.
         *
         * This is intended for python users, which will expect it to work out
         * of the box without any extra fiddling.
         */
        std::vector<uint16_t> m_ids(self.columns_per_packet);
        std::vector<uint32_t> statuses(self.columns_per_packet);

        for (int icol = 0; icol < self.columns_per_packet; ++icol) {
            uint8_t* col_buf = self.nth_col(icol, packet.buf.data());
            // store for later reassignment
            m_ids[icol] = self.col_measurement_id(col_buf);
            statuses[icol] = self.col_status(col_buf);
            // overwrite with 0..columns_per_packet
            self.set_col_measurement_id(col_buf, icol);
            self.set_col_status(col_buf, 0x1);
        }

        Eigen::Map<const ouster::sdk::core::img_t<T>> field_map(
            field.data(), field.shape(0), field.shape(1));
        // eigen is trash; this extra step is extra annoying
        Eigen::Ref<const ouster::sdk::core::img_t<T>> ref = field_map;
        self.set_block(ref, field_name_str, packet.buf.data());

        // restore m_ids and statuses
        for (int icol = 0; icol < self.columns_per_packet; ++icol) {
            uint8_t* col_buf = self.nth_col(icol, packet.buf.data());
            self.set_col_measurement_id(col_buf, m_ids[icol]);
            self.set_col_status(col_buf, statuses[icol]);
        }
    }
};

template <typename Fn>
struct LambdaIter {
    Fn lambda;

    // generative output iterators return themselves
    LambdaIter& operator*() { return *this; }
    // prefix
    LambdaIter& operator++() { return *this; }
    // postfix
    LambdaIter& operator++(int) { return *this; }

    template <typename T>
    LambdaIter& operator=(T&& item) {
        lambda(item);
        return *this;
    }
};

template <typename Fn>
LambdaIter<Fn> make_lambda_iter(Fn&& f) {
    return LambdaIter<Fn>{f};
}

/*
 * Map a channel field type to a dtype
 *
 * WARNING
 * Has only partial support of fixed width string fields.
 * Proper handling should be done outside.
 */
inline static pybind11::dtype dtype_of_field_type(
    const ouster::sdk::core::ChanFieldType& ftype) {
    switch (ftype) {
        case ouster::sdk::core::ChanFieldType::UINT8:
            return pybind11::dtype::of<uint8_t>();
        case ouster::sdk::core::ChanFieldType::UINT16:
            return pybind11::dtype::of<uint16_t>();
        case ouster::sdk::core::ChanFieldType::UINT32:
            return pybind11::dtype::of<uint32_t>();
        case ouster::sdk::core::ChanFieldType::UINT64:
            return pybind11::dtype::of<uint64_t>();
        case ouster::sdk::core::ChanFieldType::INT8:
            return pybind11::dtype::of<int8_t>();
        case ouster::sdk::core::ChanFieldType::INT16:
            return pybind11::dtype::of<int16_t>();
        case ouster::sdk::core::ChanFieldType::INT32:
            return pybind11::dtype::of<int32_t>();
        case ouster::sdk::core::ChanFieldType::INT64:
            return pybind11::dtype::of<int64_t>();
        case ouster::sdk::core::ChanFieldType::FLOAT32:
            return pybind11::dtype::of<float>();
        case ouster::sdk::core::ChanFieldType::FLOAT64:
            return pybind11::dtype::of<double>();
        case ouster::sdk::core::ChanFieldType::ZONE_STATE:
            return pybind11::dtype::of<ouster::sdk::core::ZoneState>();
        case ouster::sdk::core::ChanFieldType::CHAR:
            return pybind11::dtype("S1");
        default:
            throw std::invalid_argument(
                "Invalid field_type for conversion to dtype");
    }
    return pybind11::dtype();  // unreachable ...
}

/*
 * Map a dtype to a channel field type
 */
inline static ouster::sdk::core::ChanFieldType cft_of_dtype(
    const pybind11::dtype& dtype) {
    if (dtype.is(pybind11::dtype::of<uint8_t>())) {
        return ouster::sdk::core::ChanFieldType::UINT8;
    } else if (dtype.is(pybind11::dtype::of<uint16_t>())) {
        return ouster::sdk::core::ChanFieldType::UINT16;
    } else if (dtype.is(pybind11::dtype::of<uint32_t>())) {
        return ouster::sdk::core::ChanFieldType::UINT32;
    } else if (dtype.is(pybind11::dtype::of<uint64_t>())) {
        return ouster::sdk::core::ChanFieldType::UINT64;
    } else if (dtype.is(pybind11::dtype::of<int8_t>())) {
        return ouster::sdk::core::ChanFieldType::INT8;
    } else if (dtype.is(pybind11::dtype::of<int16_t>())) {
        return ouster::sdk::core::ChanFieldType::INT16;
    } else if (dtype.is(pybind11::dtype::of<int32_t>())) {
        return ouster::sdk::core::ChanFieldType::INT32;
    } else if (dtype.is(pybind11::dtype::of<int64_t>())) {
        return ouster::sdk::core::ChanFieldType::INT64;
    } else if (dtype.is(pybind11::dtype::of<float>())) {
        return ouster::sdk::core::ChanFieldType::FLOAT32;
    } else if (dtype.is(pybind11::dtype::of<double>())) {
        return ouster::sdk::core::ChanFieldType::FLOAT64;
    } else if (dtype.is(pybind11::dtype::of<ouster::sdk::core::ZoneState>())) {
        return ouster::sdk::core::ChanFieldType::ZONE_STATE;
    } else if (dtype.char_() == 'S') {
        return ouster::sdk::core::ChanFieldType::CHAR;
    } else {
        throw std::invalid_argument("Invalid dtype for a channel field");
    }
}

inline pybind11::object field_to_pyobj(ouster::sdk::core::Field& field,
                                       pybind11::handle handle) {
    std::vector<size_t> shape = field.shape();
    if (shape.empty()) {
        throw std::invalid_argument("Field is not an array");
    }

    pybind11::dtype dtype = dtype_of_field_type(field.tag());

    // handle array of strings
    if (field.tag() == ouster::sdk::core::ChanFieldType::CHAR) {
        ssize_t string_width = shape.back();
        // encode the last dimension into the dtype
        dtype =
            pybind11::dtype(std::string("S") + std::to_string(string_width));

        // pop the last dimension
        shape.pop_back();
        // if it was the last dimension, add one
        if (shape.empty()) {
            shape.push_back(1);
        }
    }

    auto py_shape = pybind11::array::ShapeContainer(shape.begin(), shape.end());

    pybind11::object out =
        pybind11::array(dtype, py_shape, field.get(), handle);

    if (field.tag() == ouster::sdk::core::ChanFieldType::ZONE_STATE) {
        // This just lets us use nice accessors in python, e.g.
        //     field.parent_id[1] = 100
        // instead of default array api which only allows for
        //     field['parent_id'][1] = 100
        pybind11::module_ np = pybind11::module_::import("numpy");
        out = out.attr("view")(np.attr("recarray"));
    }

    return out;
}

inline ouster::sdk::core::FieldDescriptor pyarray_to_descriptor(
    const pybind11::array& data) {
    std::vector<size_t> shape;
    shape.reserve(data.ndim());
    for (int i = 0; i < data.ndim(); i++) {
        shape.push_back(data.shape(i));
    }

    auto dtype = data.dtype();
    ouster::sdk::core::ChanFieldType cft = cft_of_dtype(dtype);

    // handle strings
    if (cft == ouster::sdk::core::ChanFieldType::CHAR && dtype.itemsize() > 0) {
        shape.push_back(static_cast<size_t>(dtype.itemsize()));
    }

    return ouster::sdk::core::FieldDescriptor::array(cft, shape);
}

inline ouster::sdk::core::FieldType init_field_type(
    const std::string& name, pybind11::dtype dtype,
    std::vector<size_t> extra_dims, ouster::sdk::core::FieldClass field_class) {
    ouster::sdk::core::ChanFieldType cft = cft_of_dtype(dtype);

    if (cft == ouster::sdk::core::ChanFieldType::CHAR && dtype.itemsize() > 0) {
        extra_dims.push_back(static_cast<size_t>(dtype.itemsize()));
    }

    return ouster::sdk::core::FieldType(name, cft, extra_dims, field_class);
}

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

/*
 * Define an enum from a table of strings, along with some properties to make
 * the class behave more like a Python enum
 */
template <typename C, typename E, size_t N>
void def_enum(C& enum_val, const Table<E, const char*, N>& strings_table,
              const std::string& enum_prefix = "") {
    // weed out empty profiles
    auto end = std::find_if(strings_table.begin(), strings_table.end(),
                            [](const std::pair<E, const char*>& prof_pair) {
                                return prof_pair.second == nullptr;
                            });

    // in pybind11 2.0, calling enum.value(const char* name, val) doesn't make a
    // copy of the name argument. When value names aren't statically allocated,
    // we have to keep them alive. Use deque for stability of c_str() pointers
    static std::deque<std::string> enumerator_names;

    // module imports
    pybind11::object mapping_proxy =
        pybind11::module::import("types").attr("MappingProxyType");

    // declare enumerators
    for (auto it = strings_table.begin(); it != end; ++it) {
        std::string underscore = isdigit(it->second[0]) ? "_" : "";
        enumerator_names.push_back(underscore + it->second);
        enum_val.value(enumerator_names.back().c_str(), it->first);

        // Map the prefixed enum to the same value (no distinction possible
        // because of this)
        if (!enum_prefix.empty()) {
            enumerator_names.push_back(enum_prefix + it->second);
            enum_val.value(enumerator_names.back().c_str(), it->first);
        }
    }

    // use immutable mapping_proxy to return members dict
    std::map<std::string, E> members;
    for (auto it = strings_table.begin(); it != end; ++it) {
        std::string underscore = isdigit(it->second[0]) ? "_" : "";
        members[underscore + it->second] = it->first;

        // Map the prefixed enum to the same value (no distinction possible
        // because of this)
        if (!enum_prefix.empty()) {
            members[enum_prefix + it->second] = it->first;
        }
    }

    pybind11::object py_members = mapping_proxy(members);
    enum_val.def_property_readonly_static(
        "__members__", [=](pybind11::object) { return py_members; },
        "Returns a mapping of member name->value.");

    // can't make the class iterable itself easily
    enum_val.def_property_readonly_static(
        "values",
        [&, end](pybind11::object) {
            return pybind11::make_key_iterator(strings_table.begin(), end);
        },
        "Returns an iterator of all enum members.");

    // support name / value properties like regular enums
    enum_val.def_property_readonly(
        "value", [](const E& self) { return static_cast<int>(self); },
        "The value of the Enum member.");
    enum_val.def_property_readonly(
        "name", [](const E& self) { return to_string(self); },
        "The name of the Enum member.");
    enum_val.attr("__str__") = pybind11::cpp_function(
        [](const E& enum_member) { return to_string(enum_member); },
        pybind11::name("__str__"), pybind11::is_method(enum_val));

    enum_val.def_static(
        "from_string",
        [=](const std::string& str) {
            return members.count(str) > 0 ? pybind11::cast(members.at(str))
                                          : pybind11::none();
        },
        "Create enum value from string.");
}
