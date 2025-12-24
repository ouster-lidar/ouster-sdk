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
#include "ouster/zone_monitor.h"

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pyerrors.h>
#include <warnings.h>

#include "client_common.h"
#include "ouster/coord.h"
#include "ouster/mesh.h"
#include "ouster/packet.h"
#include "ouster/triangle.h"
#include "ouster/types.h"
#include "ouster/zone.h"

namespace py = pybind11;
using ouster::sdk::core::mat4d;
using ouster::sdk::core::Packet;
using ouster::sdk::core::Stl;
using ouster::sdk::core::Zone;
using ouster::sdk::core::ZonePacket;
using ouster::sdk::core::ZoneSet;
using ouster::sdk::core::ZoneSetOutputFilter;
using ouster::sdk::core::ZoneState;
using ouster::sdk::core::Zrb;

PYBIND11_MAKE_OPAQUE(std::unordered_map<uint32_t, Zone>);

void init_client_zone_monitor(py::module& module, py::module& /*unused*/) {
    PYBIND11_NUMPY_DTYPE(ZoneState, live, id, error_flags, trigger_type,
                         trigger_status, triggered_frames, count,
                         occlusion_count, invalid_count, max_count, min_range,
                         max_range, mean_range);

    py::class_<ZoneState>(module, "ZoneState")
        // TODO[tws] add arg names and docstrings
        .def_readwrite("live", &ZoneState::live)
        .def_readwrite("id", &ZoneState::id)
        .def_readwrite("error_flags", &ZoneState::error_flags)
        .def_readwrite("trigger_type", &ZoneState::trigger_type)
        .def_readwrite("trigger_status", &ZoneState::trigger_status)
        .def_readwrite("triggered_frames", &ZoneState::triggered_frames)
        .def_readwrite("count", &ZoneState::count)
        .def_readwrite("occlusion_count", &ZoneState::occlusion_count)
        .def_readwrite("invalid_count", &ZoneState::invalid_count)
        .def_readwrite("max_count", &ZoneState::max_count)
        .def_readwrite("min_range", &ZoneState::min_range)
        .def_readwrite("max_range", &ZoneState::max_range)
        .def_readwrite("mean_range", &ZoneState::mean_range)
        .def_static("dtype", []() { return py::dtype::of<ZoneState>(); });

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
        .def_readwrite("coordinate_frame", &Stl::coordinate_frame)
        .def_readwrite("filename", &Stl::filename)
        .def("to_mesh", &Stl::to_mesh)
        .def_property_readonly("hash",
                               [](Stl& self) { return self.hash().str(); })
        .def("__eq__",
             [](const Stl& lhs, const Stl& rhs) { return lhs == rhs; });

    py::class_<Zrb>(module, "Zrb")
        // TODO[tws] add arg names and docstrings
        .def(py::init<>())
        .def(py::init<std::string>())
        .def(py::init<const std::vector<uint8_t>&>())
        .def_readwrite("serial_number", &Zrb::serial_number)
        .def_property_readonly("stl_hash",
                               [](Zrb& self) -> nonstd::optional<std::string> {
                                   if (!self.stl_hash) {
                                       return nonstd::nullopt;
                                   }
                                   return self.stl_hash->str();
                               })
        .def_readwrite("beam_to_lidar_transform", &Zrb::beam_to_lidar_transform)
        .def_readwrite("lidar_to_sensor_transform",
                       &Zrb::lidar_to_sensor_transform)
        .def_readwrite("sensor_to_body_transform",
                       &Zrb::sensor_to_body_transform)
        .def_property(
            "near_range_mm",
            [](Zrb& self) -> ouster::sdk::core::img_t<uint32_t>& {
                return self.near_range_mm;
            },
            [](Zrb& self, const ouster::sdk::core::img_t<uint32_t>& img) {
                self.near_range_mm = img;
            })

        .def_property(
            "far_range_mm",
            [](Zrb& self) -> ouster::sdk::core::img_t<uint32_t>& {
                return self.far_range_mm;
            },
            [](Zrb& self, const ouster::sdk::core::img_t<uint32_t>& img) {
                self.far_range_mm = img;
            })

        .def_property_readonly("hash",
                               [](Zrb& self) { return self.hash().str(); })
        .def("blob", &Zrb::blob)
        .def("save",
             py::overload_cast<const std::string&>(&Zrb::save, py::const_))
        .def("__eq__",
             [](const Zrb& lhs, const Zrb& rhs) { return lhs == rhs; });

    py::class_<Zone>(module, "Zone")
        // TODO[tws] add arg names and docstrings
        .def(py::init<>())
        .def_readwrite("point_count", &Zone::point_count)
        .def_readwrite("frame_count", &Zone::frame_count)
        .def_readwrite("mode", &Zone::mode)
        .def_readwrite("stl", &Zone::stl)
        .def_readwrite("zrb", &Zone::zrb)
        .def_readwrite("label", &Zone::label)
        //.def("render", &Zone::render)  // TODO[tws] either bind BeamConfig or
        // accept a SensorInfo
        .def("__eq__",
             [](const Zone& lhs, const Zone& rhs) { return lhs == rhs; });

    py::bind_map<std::unordered_map<uint32_t, Zone>>(module,
                                                     "AvailableZonesMap");

    py::enum_<ZoneSetOutputFilter>(module, "ZoneSetOutputFilter")
        .value("STL", ZoneSetOutputFilter::STL)
        .value("ZRB", ZoneSetOutputFilter::ZRB)
        .value("STL_AND_ZRB", ZoneSetOutputFilter::STL_AND_ZRB);

    py::class_<ZoneSet>(module, "ZoneSet")
        .def(py::init<>())
        .def(
            py::init([](const py::str& zip_path) { return ZoneSet(zip_path); }))
        .def(py::init([](const py::bytes& zip_bytes) {
            py::buffer_info buf_info = py::buffer(zip_bytes).request();
            uint8_t* buf_ptr = static_cast<uint8_t*>(buf_info.ptr);
            std::vector<uint8_t> zip_vec(buf_ptr, buf_ptr + buf_info.size);
            return ZoneSet(zip_vec);
        }))
        .def_property(
            "zones",
            [](ZoneSet& self) -> std::unordered_map<uint32_t, Zone>& {
                return self.zones;
            },
            [](ZoneSet& self, const py::dict& value) {
                for (std::pair<py::handle, py::handle> item : value) {
                    uint32_t key = item.first.cast<uint32_t>();
                    Zone zone = item.second.cast<Zone>();
                    self.zones[key] = zone;
                }
            })
        .def_readwrite("power_on_live_ids", &ZoneSet::power_on_live_ids)
        .def_readwrite("sensor_to_body_transform",
                       &ZoneSet::sensor_to_body_transform)
        .def_readwrite("label", &ZoneSet::label)
        .def("save", &ZoneSet::save)
        .def("render", &ZoneSet::render)
        .def("to_zip_blob",
             [](ZoneSet& self, ZoneSetOutputFilter filter) {
                 std::vector<uint8_t> vec = self.to_zip_blob(filter);
                 return py::bytes(reinterpret_cast<const char*>(vec.data()),
                                  vec.size());
             })
        .def("to_json", &ZoneSet::to_json)
        .def("__eq__",
             [](const ZoneSet& lhs, const ZoneSet& rhs) { return lhs == rhs; });

    py::class_<ZonePacket, Packet, std::shared_ptr<ZonePacket>>(module,
                                                                "ZonePacket")
        .def(py::init<int>(), py::arg("size") = 65536)
        .def("__copy__",
             [](const ZonePacket& self) { return ZonePacket(self); })
        .def("__deepcopy__", [](const ZonePacket& self, const py::dict&) {
            return ZonePacket(self);
        });
}
