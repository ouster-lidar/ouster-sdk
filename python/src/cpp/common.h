#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <cstdint>
#include <functional>
#include <map>

#include "eigen_dense.h"
#include "nonstd/optional.hpp"
#include "ouster/core/open_source.h"

template <class IterT>
struct iterator_holder {
    IterT iter, end;
    bool first_or_done = true;
};

namespace py = nanobind;

namespace nanobind::detail {
template <>
struct dtype_traits<ouster::sdk::core::float16_t> {
    static constexpr dlpack::dtype value{
        (uint8_t)dlpack::dtype_code::Float,  // type code
        16,                                  // size in bits
        1                                    // lanes (simd), usually set to 1
    };
    static constexpr auto name = const_name("float16");
};
}  // namespace nanobind::detail

/**
 * Helper macro to define read-write optional properties
 *
 * @param bind_class The nanobind class_ to bind the property to
 * @param obj_type The C++ object type containing the member
 * @param name The name of the property
 * @param member The member variable name
 * @param doc The docstring for the property
 * @param ... Additional nanobind arguments
 */
#define def_opt(bind_class, obj_type, name, member, doc, ...)                         \
    (bind_class)                                                                      \
        .def_prop_rw(                                                                 \
            name, [](const obj_type& obj) { return obj.member; },                     \
            [](obj_type& obj, const decltype(obj.member)& val) { obj.member = val; }, \
            py::arg().none(), doc, ##__VA_ARGS__);

/**
 * Helper macro to define read-write optional properties
 * while avoiding a conversion.
 *
 * @param bind_class The nanobind class_ to bind the property to
 * @param obj_type The C++ object type containing the member
 * @param name The name of the property
 * @param member The member variable name
 * @param doc The docstring for the property
 * @param ... Additional nanobind arguments
 */
#define def_opt_noconvert(bind_class, obj_type, name, member, doc, ...)               \
    (bind_class)                                                                      \
        .def_prop_rw(                                                                 \
            name, [](const obj_type& obj) { return obj.member; },                     \
            [](obj_type& obj, const decltype(obj.member)& val) { obj.member = val; }, \
            py::arg().none().noconvert(), doc, ##__VA_ARGS__);

void parse_packet_source_options(const py::kwargs& args, ouster::sdk::PacketSourceOptions& options);
void parse_frame_set_source_options(const py::kwargs& args,
                                    ouster::sdk::FrameSetSourceOptions& options);

namespace nanobind::detail {

template <typename T>
struct type_caster<nonstd::optional<T>> {
    using Caster = make_caster<T>;

    static constexpr auto Name = const_name("typing.Optional[") + Caster::Name + const_name("]");

    template <typename T_>
    using Cast = nonstd::optional<T>;

    static handle from_cpp(const nonstd::optional<T>& src, rv_policy policy,
                           cleanup_list* cleanup) noexcept {
        if (!src) {
            return none().release();
        }
        return Caster::from_cpp(*src, policy, cleanup);
    }

    bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept {
        if (src.is_none()) {
            value = nonstd::nullopt;
            return true;
        }

        Caster caster;
        if (caster.from_python(src, flags, cleanup)) {
            value.emplace(caster.operator cast_t<T>());
            return true;
        }
        return false;
    }

    operator nonstd::optional<T>*() {
        return &value;
    }
    operator nonstd::optional<T>&() {
        return value;
    }
    operator nonstd::optional<T>&&() {
        return std::move(value);
    }
    operator nonstd::optional<T>() {
        return std::move(value);
    }

    nonstd::optional<T> value;
};
};  // namespace nanobind::detail

class OusterDtype {
   public:
    OusterDtype(const ouster::sdk::core::ChanFieldType& ftype);
    OusterDtype(const py::dlpack::dtype& dtype);
    OusterDtype(const std::string& stype);
    OusterDtype(const py::object& otype);

    py::dlpack::dtype dtype() const;
    ouster::sdk::core::ChanFieldType cft() const;
    std::string stype() const;
    py::object otype() const;
    bool is_numeric() const;
    bool is_string() const;
    bool is_object() const;

   protected:
    std::string stype_;
    void from(const ouster::sdk::core::ChanFieldType& ftype);
    void from(const py::dlpack::dtype& dtype);
    void from(const std::string& stype);
    void from(const py::object& otype);
    bool _numeric;
    bool _string;
    bool _object;
};
