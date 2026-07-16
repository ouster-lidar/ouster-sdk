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
#include "ouster/core/zone_monitor.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/bind_map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>
#include <pyerrors.h>
#include <warnings.h>

#include "client_common.h"
#include "common.h"
#include "eigen_dense.h"
#include "ouster/core/coord.h"
#include "ouster/core/mesh.h"
#include "ouster/core/packet.h"
#include "ouster/core/triangle.h"
#include "ouster/core/types.h"
#include "ouster/core/zone.h"

using ouster::sdk::core::mat4d;
using ouster::sdk::core::Packet;
using ouster::sdk::core::PacketFormat;
using ouster::sdk::core::Stl;
using ouster::sdk::core::Zone;
using ouster::sdk::core::ZonePacket;
using ouster::sdk::core::ZoneSet;
using ouster::sdk::core::ZoneSetOutputFilter;
using ouster::sdk::core::ZoneState;
using ouster::sdk::core::Zrb;

NB_MAKE_OPAQUE(std::unordered_map<uint32_t, Zone>);

void init_client_zone_monitor(py::module_& module, py::module_& /*unused*/) {
    py::class_<ZoneState>(module, "ZoneState")
        // TODO[tws] add arg names and docstrings
        .def_rw("live", &ZoneState::live)
        .def_rw("id", &ZoneState::id)
        .def_rw("error_flags", &ZoneState::error_flags)
        .def_rw("trigger_type", &ZoneState::trigger_type)
        .def_rw("trigger_status", &ZoneState::trigger_status)
        .def_rw("triggered_frames", &ZoneState::triggered_frames)
        .def_rw("count", &ZoneState::count)
        .def_rw("occlusion_count", &ZoneState::occlusion_count)
        .def_rw("invalid_count", &ZoneState::invalid_count)
        .def_rw("max_count", &ZoneState::max_count)
        .def_rw("min_range", &ZoneState::min_range)
        .def_rw("max_range", &ZoneState::max_range)
        .def_rw("mean_range", &ZoneState::mean_range)
        .def_static("dtype", []() {
            OusterDtype dtype(ouster::sdk::core::ChanFieldType::ZONE_STATE);
            return dtype.otype();
        });

    py::enum_<Zone::ZoneMode>(module, "ZoneMode")
        .value("NONE", Zone::ZoneMode::NONE)
        .value("OCCUPANCY", Zone::ZoneMode::OCCUPANCY)
        .value("VACANCY", Zone::ZoneMode::VACANCY);

    py::enum_<Stl::CoordinateFrame>(module, "CoordinateFrame")
        .value("BODY", Stl::CoordinateFrame::BODY)
        .value("SENSOR", Stl::CoordinateFrame::SENSOR);

    py::class_<Stl>(module, "Stl")
        // TODO[tws] add arg names and docstrings
        .def(py::init<std::string>())
        .def(py::init<const std::vector<uint8_t>&>())
        .def_rw("coordinate_frame", &Stl::coordinate_frame)
        .def_rw("filename", &Stl::filename)
        .def("to_mesh", &Stl::to_mesh)
        .def_prop_ro("hash", [](Stl& self) { return self.hash().str(); })
        .def("__eq__", [](const Stl& lhs, const py::object& rhs) {
            if (!py::isinstance<Stl>(rhs)) {
                return false;
            }
            return lhs == py::cast<const Stl&>(rhs);
        });

    py::class_<Zrb>(module, "Zrb")
        // TODO[tws] add arg names and docstrings
        .def(py::init<>())
        .def(py::init<std::string>())
        .def(py::init<const std::vector<uint8_t>&>())
        .def_rw("serial_number", &Zrb::serial_number)
        .def_prop_ro("stl_hash",
                     [](Zrb& self) -> py::object {
                         if (!self.stl_hash) {
                             return py::none();
                         }
                         return py::cast(self.stl_hash->str());
                     })
        .def_rw("beam_to_lidar_transform", &Zrb::beam_to_lidar_transform)
        .def_rw("lidar_to_sensor_transform", &Zrb::lidar_to_sensor_transform)
        .def_rw("sensor_to_body_transform", &Zrb::sensor_to_body_transform)
        .def_prop_rw(
            "near_range_mm",
            [](Zrb& self) -> ouster::sdk::core::img_t<uint32_t>& { return self.near_range_mm; },
            [](Zrb& self, const ouster::sdk::core::img_t<uint32_t>& img) {
                self.near_range_mm = img;
            },
            py::rv_policy::reference_internal)

        .def_prop_rw(
            "far_range_mm",
            [](Zrb& self) -> ouster::sdk::core::img_t<uint32_t>& { return self.far_range_mm; },
            [](Zrb& self, const ouster::sdk::core::img_t<uint32_t>& img) {
                self.far_range_mm = img;
            },
            py::rv_policy::reference_internal)

        .def_prop_ro("hash", [](Zrb& self) { return self.hash().str(); })
        .def("blob", &Zrb::blob)
        .def("save", py::overload_cast<const std::string&>(&Zrb::save, py::const_))
        .def("__eq__", [](const Zrb& lhs, const py::object& rhs) {
            if (!py::isinstance<Zrb>(rhs)) {
                return false;
            }
            return lhs == py::cast<const Zrb&>(rhs);
        });

    auto zone_object = py::class_<Zone>(module, "Zone")
                           // TODO[tws] add arg names and docstrings
                           .def(py::init<>())
                           .def_rw("point_count", &Zone::point_count)
                           .def_rw("frame_count", &Zone::frame_count)
                           .def_rw("mode", &Zone::mode)
                           .def_rw("label", &Zone::label)
                           //.def("render", &Zone::render)  // TODO[tws] either
                           // bind BeamConfig or
                           // accept a SensorInfo
                           .def("__eq__", [](const Zone& lhs, const py::object& rhs) {
                               if (!py::isinstance<Zone>(rhs)) {
                                   return false;
                               }
                               return lhs == py::cast<const Zone&>(rhs);
                           });
    zone_object.def_prop_rw(
        "stl",
        [](const Zone* obj) {
            if (obj->stl.has_value()) {
                return py::cast(obj->stl.value(), py::rv_policy::reference_internal, py::cast(obj));
            }
            return py::none();
        },
        [](Zone& obj, const decltype(Zone::stl)& val) { obj.stl = val; },
        py::for_getter(py::sig("def stl(self, /) -> Optional[Stl]")), py::arg().none(), "",
        py::rv_policy::reference_internal);
    zone_object.def_prop_rw(
        "zrb",
        [](const Zone* obj) {
            if (obj->zrb.has_value()) {
                return py::cast(obj->zrb.value(), py::rv_policy::reference_internal, py::cast(obj));
            }
            return py::none();
        },
        [](Zone& obj, const decltype(Zone::zrb)& val) { obj.zrb = val; }, py::arg().none(),
        py::for_getter(py::sig("def zrb(self, /) -> Optional[Zrb]")), "",
        py::rv_policy::reference_internal);

    // specify return policy to allow mutability
    py::bind_map<std::unordered_map<uint32_t, Zone>, py::rv_policy::reference_internal>(
        module, "AvailableZonesMap");

    py::enum_<ZoneSetOutputFilter>(module, "ZoneSetOutputFilter")
        .value("STL", ZoneSetOutputFilter::STL)
        .value("ZRB", ZoneSetOutputFilter::ZRB)
        .value("STL_AND_ZRB", ZoneSetOutputFilter::STL_AND_ZRB);

    py::class_<ZoneSet>(module, "ZoneSet")
        .def(py::init<>())
        // Nanobind constructors using lambdas must perform placement new
        .def("__init__",
             [](ZoneSet* self, const py::str& zip_path) {
                 new (self) ZoneSet(py::cast<std::string>(zip_path));
             })
        .def("__init__",
             [](ZoneSet* self, const py::bytes& zip_bytes) {
                 const uint8_t* buf_ptr = static_cast<const uint8_t*>(zip_bytes.data());
                 std::vector<uint8_t> zip_vec(buf_ptr, buf_ptr + zip_bytes.size());
                 new (self) ZoneSet(zip_vec);
             })
        .def_prop_rw(
            "zones",
            [](ZoneSet& self) -> std::unordered_map<uint32_t, Zone>* { return &self.zones; },
            [](ZoneSet& self, const py::dict& value) {
                for (auto item : value) {
                    uint32_t key = py::cast<uint32_t>(item.first);
                    Zone zone = py::cast<Zone>(item.second);
                    self.zones[key] = zone;
                }
            },
            py::rv_policy::reference_internal)
        .def_rw("power_on_live_ids", &ZoneSet::power_on_live_ids)
        .def_prop_rw(
            "sensor_to_body_transform",
            [](const ZoneSet& self) {
                if (!self.sensor_to_body_transform.has_value()) {
                    return py::none();
                }
                return py::cast(self.sensor_to_body_transform.value());
            },
            [](ZoneSet& self, const py::object& value) {
                if (value.is_none()) {
                    self.sensor_to_body_transform = {};
                    return;
                }
                self.sensor_to_body_transform = py::cast<mat4d>(value);
            },
            py::rv_policy::reference_internal)
        .def_rw("label", &ZoneSet::label)
        .def("save", &ZoneSet::save)
        .def("save_to_directory", &ZoneSet::save_to_directory)
        .def("render", &ZoneSet::render)
        .def("to_zip_blob",
             [](ZoneSet& self, ZoneSetOutputFilter filter) {
                 std::vector<uint8_t> vec = self.to_zip_blob(filter);
                 return py::bytes(reinterpret_cast<const char*>(vec.data()), vec.size());
             })
        .def("to_json", &ZoneSet::to_json)
        .def("__eq__", [](const ZoneSet& lhs, const py::object& rhs) {
            if (!py::isinstance<ZoneSet>(rhs)) {
                return false;
            }
            return lhs == py::cast<const ZoneSet&>(rhs);
        });

    py::class_<ZonePacket, Packet>(module, "ZonePacket")
        .def(
            "__init__",
            [](ZonePacket* self, int size = 65536) {
                PyErr_WarnEx(PyExc_FutureWarning,
                             "ZonePacket(size: int) is deprecated, use "
                             "ZonePacket(fmt: PacketFormat) instead",
                             1);
                new (self) ZonePacket(size);
            },
            py::arg("size") = 65536)
        .def(py::init<std::shared_ptr<PacketFormat>>())
        .def("__copy__", [](const ZonePacket& self) { return ZonePacket(self); })
        .def("__deepcopy__",
             [](const ZonePacket& self, const py::dict&) { return ZonePacket(self); });
}
