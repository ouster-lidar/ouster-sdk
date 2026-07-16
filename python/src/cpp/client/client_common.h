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
#include <eigen_dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <cstdint>
#include <functional>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "common.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/packet.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/types.h"
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
const uint8_t* getptr(size_t bound, const py::object& buf);

template <typename T>
struct SetField {
    // Use generic ndarray with c_contig constraint
    using Field = py::ndarray<T, py::c_contig>;

    void operator()(const ouster::sdk::core::PacketFormat& self,
                    ouster::sdk::core::LidarPacket& packet, const std::string& field_name_str,
                    Field field) const {
        if (field.ndim() != 2 || field.shape(0) != static_cast<size_t>(self.pixels_per_column) ||
            field.shape(1) != static_cast<size_t>(self.columns_per_packet)) {
            throw std::invalid_argument("field dimension mismatch");
        }

        /**
         * It is a bit weird to be setting these back and forth to work around
         * PacketFormat::set_block logic that is intended for lidar frame usage
         * but I do think it is better than keeping two versions of the same.
         *
         * This is intended for python users, which will expect it to work out
         * of the box without any extra fiddling.
         */
        std::vector<uint16_t> m_ids(self.columns_per_packet);
        std::vector<uint32_t> statuses(self.columns_per_packet);

        for (uint32_t icol = 0; icol < self.columns_per_packet; ++icol) {
            uint8_t* col_buf = self.nth_col(icol, packet.buf.data());
            // store for later reassignment
            m_ids[icol] = self.col_measurement_id(col_buf);
            statuses[icol] = self.col_status(col_buf);
            // overwrite with 0..columns_per_packet
            self.set_col_measurement_id(col_buf, icol);
            self.set_col_status(col_buf, 0x1);
        }

        Eigen::Map<const ouster::sdk::core::img_t<T>> field_map(field.data(), field.shape(0),
                                                                field.shape(1));
        // eigen is trash; this extra step is extra annoying
        Eigen::Ref<const ouster::sdk::core::img_t<T>> ref = field_map;
        self.set_block(ref.data(), ref.cols(), field_name_str, packet.buf.data());

        // restore m_ids and statuses
        for (uint32_t icol = 0; icol < self.columns_per_packet; ++icol) {
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
    LambdaIter& operator*() {
        return *this;
    }
    // prefix
    LambdaIter& operator++() {
        return *this;
    }
    // postfix
    LambdaIter& operator++(int) {
        return *this;
    }

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

py::object field_to_pyobj(ouster::sdk::core::FieldView& field, py::handle handle);

ouster::sdk::core::FieldDescriptor pyarray_to_descriptor(const py::object& data);

ouster::sdk::core::FieldType init_field_type(const std::string& name, const py::object& dtype,
                                             std::vector<size_t> extra_dims,
                                             ouster::sdk::core::FieldClass field_class);

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

template <typename C, typename E, size_t N>
std::map<std::string, E> populate_enum(C& enum_val, const Table<E, const char*, N>& strings_table) {
    std::map<std::string, E> members;
    auto pair_filter = [](const std::pair<E, const char*>& prof_pair) {
        return prof_pair.second == nullptr;
    };

    auto end = std::find_if(strings_table.begin(), strings_table.end(), pair_filter);
    // declare enumerators
    for (auto it = strings_table.begin(); it != end; ++it) {
        std::string underscore = isdigit(it->second[0]) ? "_" : "";
        std::string name = underscore + it->second;

        // 1. Register the standard name
        enum_val.value(name.c_str(), it->first);
        members[name] = it->first;
    }

    return members;
}

/*
 * Define an enum from a table of strings, along with some properties to make
 * the class behave more like a Python enum
 */
template <typename C, typename E, size_t N>
void def_enum(C& enum_val, const Table<E, const char*, N>& strings_table,
              const std::string& enum_name) {
    // Build a local C++ map for the 'from_string' implementation below
    std::map<std::string, E> members = populate_enum(enum_val, strings_table);

    auto pair_filter = [](const std::pair<E, const char*>& prof_pair) {
        return prof_pair.second == nullptr;
    };

    enum_val.def_static(
        "values",
        [=]() {
            std::vector<E> values_list;
            auto end = std::find_if(strings_table.begin(), strings_table.end(), pair_filter);
            values_list.reserve(std::distance(strings_table.begin(), end));
            for (auto it = strings_table.begin(); it != end; ++it) {
                values_list.push_back(it->first);
            }
            return values_list;
        },
        "Returns a list of all enum members.");

    // Regular read-only properties for instances
    enum_val.def_prop_ro(
        "value", [](const E& self) { return static_cast<int>(self); },
        "The value of the Enum member.");

    enum_val.def(
        "__int__", [](const E& self) { return static_cast<int>(self); },
        "The value of the Enum member.");

    enum_val.def("__str__", [](const E& enum_member) { return to_string(enum_member); });

    // Static from_string helper
    // This captures the 'members' map by value so it persists
    // this is a leak but its fine:
    std::string cppsig = "def from_string(string: str) -> typing.Optional[" + enum_name + "]";
    const char* sig = strdup(cppsig.c_str());
    enum_val.def_static(
        "from_string",
        [members](const std::string& str) -> py::object {
            if (members.count(str) > 0) {
                return py::cast(members.at(str));
            }
            return py::none();
        },
        "Create enum value from string.", py::sig(sig));
}
