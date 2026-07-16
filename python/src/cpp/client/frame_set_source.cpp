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
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/bind_map.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>
#include <pyerrors.h>
#include <warnings.h>

#include <utility>

#include "common.h"
#include "eigen_dense.h"
#include "ouster/core/defaults.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/packet_source.h"
#include "ouster/core/types.h"
#include "ouster/osf/osf_frame_set_source.h"
#include "ouster/pcap/pcap_frame_set_source.h"
#include "ouster/sensor/sensor_frame_set_source.h"
#include "ouster/sensor/sensor_packet_source.h"

using namespace py::literals;
using ouster::sdk::FrameSetSourceOptions;
using ouster::sdk::PacketSourceOptions;
using ouster::sdk::core::ClassMap;
using ouster::sdk::core::ClassMapSet;
using ouster::sdk::core::FieldType;
using ouster::sdk::core::FrameSet;
using ouster::sdk::core::FrameSetSource;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::sensor::Sensor;
using ouster::sdk::sensor::SensorFrameSetSource;
using ouster::sdk::sensor::SensorPacketSource;

namespace {
std::map<void*, py::object> holders;
}  // namespace

class PyPacketSource : public ouster::sdk::core::PacketSource {
   public:
    NB_TRAMPOLINE(ouster::sdk::core::PacketSource, 1);

    mutable std::vector<std::shared_ptr<SensorInfo>>
        sensor_info_;  // NOLINT(readability-identifier-naming)

    PyPacketSource() = default;

    // dummy
    ouster::sdk::core::PacketIterator begin() const override {
        return ouster::sdk::core::PacketIterator(this);
    }

    void close() override {
        NB_OVERRIDE_PURE(close);
    }

    bool is_live() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(static_cast<const ouster::sdk::core::PacketSource*>(this));
        auto res = self.attr("is_live");
        return py::cast<bool>(res);
    }

    const std::vector<std::shared_ptr<SensorInfo>>& sensor_info() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(static_cast<const ouster::sdk::core::PacketSource*>(this));
        auto res = self.attr("sensor_info");
        sensor_info_ = py::cast<std::vector<std::shared_ptr<SensorInfo>>>(res);
        return sensor_info_;
    }
};

class PythonFrameSetIteratorImpl : public ouster::sdk::core::FrameSetIteratorImpl {
    py::iterator iterator_;

    FrameSet frame_set_;
    uint64_t desired_sn_;

   public:
    PythonFrameSetIteratorImpl(py::iterator iter, uint64_t desired_sn = 0)
        : iterator_(std::move(iter)), desired_sn_(desired_sn) {}

    ~PythonFrameSetIteratorImpl() override {
        py::gil_scoped_acquire acquire;
        iterator_ = py::iterator();
    }

    bool advance(size_t offset) override {
        py::gil_scoped_acquire acquire;
        for (size_t i = 0; i < offset; i++) {
            if (iterator_ == py::iterator::sentinel()) {
                return true;
            }
            auto value = *iterator_;
            frame_set_ = py::cast<FrameSet>(value);
            ++iterator_;

            if (frame_set_.size() != 1) {
                throw std::runtime_error("Must only yield arrays of 1 frame.");
            }

            // skip data from undesired sensors
            if (desired_sn_ != 0 && frame_set_[0]->sensor_info->sn != desired_sn_) {
                offset++;
            }
        }
        return false;
    }

    FrameSet value() override {
        return frame_set_;
    }
};

class PyFrameSetSource : public ouster::sdk::core::FrameSetSource {
    mutable std::vector<std::shared_ptr<SensorInfo>> sensor_info_;

   public:
    NB_TRAMPOLINE(ouster::sdk::core::FrameSetSource, 1);
    PyFrameSetSource() = default;

    ouster::sdk::core::FrameSetIterator begin() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(static_cast<const ouster::sdk::core::FrameSetSource*>(this));
        auto it = self.attr("__iter__")();
        return ouster::sdk::core::FrameSetIterator(
            this, new PythonFrameSetIteratorImpl(py::cast<py::iterator>(it)));
    }

    ouster::sdk::core::FrameSetIterator begin(int sensor_idx) const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(static_cast<const ouster::sdk::core::FrameSetSource*>(this));
        auto it = self.attr("__iter__")();
        auto serial_num = sensor_info()[sensor_idx]->sn;
        return ouster::sdk::core::FrameSetIterator(
            this, new PythonFrameSetIteratorImpl(py::cast<py::iterator>(it), serial_num));
    }

    bool is_live() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(static_cast<const ouster::sdk::core::FrameSetSource*>(this));
        auto res = self.attr("is_live");
        return py::cast<bool>(res);
    }

    size_t size_hint() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(static_cast<const ouster::sdk::core::FrameSetSource*>(this));
        auto res = self.attr("__length_hint__")();
        return py::cast<size_t>(res);
    }

    const std::vector<std::shared_ptr<SensorInfo>>& sensor_info() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(static_cast<const ouster::sdk::core::FrameSetSource*>(this));
        auto res = self.attr("sensor_info");
        sensor_info_ = py::cast<std::vector<std::shared_ptr<SensorInfo>>>(res);
        return sensor_info_;
    }

    // dummy
    std::unique_ptr<ouster::sdk::core::FrameSetSource> move() override {
        throw std::runtime_error("Moving not supported with this type.");
    }

   protected:
    void close() override {
        NB_OVERRIDE_PURE(close);
    }
};

class MultiWrapper : public ouster::sdk::core::MultiFrameSetSource {
   public:
    MultiWrapper(const std::vector<std::shared_ptr<ouster::sdk::core::FrameSetSource>>& srcs)
        : ouster::sdk::core::MultiFrameSetSource(srcs) {}

    ~MultiWrapper() {
        holders.erase(this);
    }
};

void init_client_frame_set_source(py::module_& module, py::module_& /*unused*/) {
    py::bind_map<std::unordered_map<std::string, ouster::sdk::core::ClassMap>>(module,
                                                                               "ClassMapDict");

    class PacketSourceHack : public ouster::sdk::core::PacketSource {
       public:
        using ouster::sdk::core::PacketSource::close;
    };

    py::class_<ouster::sdk::core::PacketSource, PyPacketSource>(module, "PacketSource",
                                                                R"(
            PacketSource is a base class for reading packet data from various sources.

            Attributes:
                SensorInfo (List[SensorInfo]): Metadata about the sensors providing the packets.
                is_live (bool): Indicates whether the packet source is live (actively receiving data).

            Methods:
                close(): Closes the packet source and releases any associated resources.
                __iter__(): Returns an iterator over the packets in the source.
            )")
        .def(py::init<>())
        .def_prop_ro("sensor_info", &ouster::sdk::core::PacketSource::sensor_info,
                     R"(
                            Retrieve sensor information for all sensors in the packet source.

                            Returns:
                                A list of `SensorInfo` objects, each containing metadata
                                about a sensor, such as serial number, firmware version,
                                and calibration details.
                            )")
        .def_prop_ro("is_live", &ouster::sdk::core::PacketSource::is_live,
                     R"(Check if the packet source is live.)")
        .def("close",
             [](ouster::sdk::core::PacketSource* self) {
                 static_cast<PacketSourceHack*>(self)->close();
             })
        .def("__enter__", [](ouster::sdk::core::PacketSource& /*self*/) {})
        .def(
            "__exit__",
            [](ouster::sdk::core::PacketSource* self, py::object& /*exc_type*/,
               py::object& /*exc_value*/,
               py::object& /*traceback*/) { static_cast<PacketSourceHack*>(self)->close(); },
            py::arg("exc_type").none(), py::arg("exc_value").none(), py::arg("traceback").none())
        .def(
            "__iter__",
            [](const ouster::sdk::core::PacketSource& source) {
                iterator_holder<ouster::sdk::core::PacketIterator> holder{source.begin(),
                                                                          source.end()};
                return holder;
            },
            py::keep_alive<0, 1>() /* Essential: keep object alive while iterator exists */,
            py::sig("def __iter__(self, /) -> Iterator[Tuple[int, "
                    "Union[LidarPacket, ImuPacket, ZonePacket]]]"));

    py::class_<iterator_holder<ouster::sdk::core::PacketIterator>>(module, "packet_iterator")
        .def("__iter__",
             [](iterator_holder<ouster::sdk::core::PacketIterator>* self) { return self; })
        .def(
            "__next__",
            [](iterator_holder<ouster::sdk::core::PacketIterator>& self) {
                if (!self.first_or_done) {
                    py::gil_scoped_release release;
                    self.iter++;
                } else {
                    self.first_or_done = false;
                }
                if (self.iter == self.end) {
                    self.first_or_done = true;
                    throw py::stop_iteration();
                }
                auto ptr = (*self.iter).second;

                // todo avoid the copies
                auto copy = new ouster::sdk::core::Packet(*ptr);
                if (ptr->type() == ouster::sdk::core::PacketType::Lidar) {
                    std::pair<int, std::shared_ptr<ouster::sdk::core::LidarPacket>> pair;
                    pair.first = (*self.iter).first;
                    pair.second.reset(static_cast<ouster::sdk::core::LidarPacket*>(copy));
                    return py::cast(pair);
                }
                if (ptr->type() == ouster::sdk::core::PacketType::Zone) {
                    std::pair<int, std::shared_ptr<ouster::sdk::core::ZonePacket>> pair;
                    pair.first = (*self.iter).first;
                    pair.second.reset(static_cast<ouster::sdk::core::ZonePacket*>(copy));
                    return py::cast(pair);
                } else {
                    std::pair<int, std::shared_ptr<ouster::sdk::core::ImuPacket>> pair;
                    pair.first = (*self.iter).first;
                    pair.second.reset(static_cast<ouster::sdk::core::ImuPacket*>(copy));
                    return py::cast(pair);
                }
            },
            py::sig("def __next__(self, /) -> Tuple[int, Union[LidarPacket, "
                    "ImuPacket, ZonePacket]]"));

    py::class_<iterator_holder<ouster::sdk::core::FrameSetIterator>>(module, "frame_set_iterator")
        .def("__iter__",
             [](iterator_holder<ouster::sdk::core::FrameSetIterator>* self) { return self; })
        .def("__next__", [](iterator_holder<ouster::sdk::core::FrameSetIterator>& self) {
            py::gil_scoped_release release;
            if (!self.first_or_done) {
                self.iter++;
            } else {
                self.first_or_done = false;
            }
            if (self.iter == self.end) {
                self.first_or_done = true;
                throw py::stop_iteration();
            }

            return *self.iter;
        });

    py::class_<SensorPacketSource, ouster::sdk::core::PacketSource>(module, "SensorPacketSource",
                                                                    R"(
        SensorPacketSource is a class for reading packet data from a sensor.

        Examples:
            - Reading packets from a sensor:
                `SensorPacketSource(sensors, config_timeout, buffer_time)`

        Args:
            sensors (List[Sensor]): A list of sensors to read packets from.
            config_timeout (float): Timeout for sensor configuration in seconds.
            buffer_time (float): Buffer time for packet storage in seconds.

        Returns:
            SensorPacketSource: An instance of the packet source for reading packets.
        )")
        .def(
            "__init__",
            [](SensorPacketSource* self, const std::vector<Sensor>& sensors, double config_timeout,
               double buffer_time) {
                new (self) SensorPacketSource(sensors, config_timeout, buffer_time);
            },
            py::arg("sensors"), py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
            py::arg("buffer_time") = 0)
        .def(
            "__init__",
            [](SensorPacketSource* self, const std::string& file, const py::kwargs& kwargs) {
                ouster::sdk::PacketSourceOptions opts;
                parse_packet_source_options(kwargs, opts);
                new (self) ouster::sdk::sensor::SensorPacketSource(file, opts);
            },
            py::arg("file"), py::arg("kwargs"))
        .def(
            "__init__",
            [](SensorPacketSource* self, const std::vector<std::string>& file,
               const py::kwargs& kwargs) {
                ouster::sdk::PacketSourceOptions opts;
                parse_packet_source_options(kwargs, opts);
                new (self) ouster::sdk::sensor::SensorPacketSource(file, opts);
            },
            py::arg("file"), py::arg("kwargs"))
        .def(
            "__init__",
            [](SensorPacketSource* self, const std::vector<Sensor>& sensors,
               const std::vector<SensorInfo>& metadata, double config_timeout, double buffer_size) {
                new (self) SensorPacketSource(sensors, metadata, config_timeout, buffer_size);
            },
            py::arg("sensors"), py::arg("metadata"),
            py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
            py::arg("buffer_time") = 0)
        .def("flush", &SensorPacketSource::flush)
        .def("buffer_size", &SensorPacketSource::buffer_size)
        .def("sockets", &SensorPacketSource::sockets)
        .def(
            "get_packet",
            [](SensorPacketSource& self, double timeout) {
                py::gil_scoped_release release;
                auto packet = self.get_packet(timeout);
                return packet;
            },
            py::arg("timeout") = 0.1)
        .def_prop_ro("dropped_packets", &SensorPacketSource::dropped_packets);

    py::class_<ouster::sdk::core::ClassMap>(module, "ClassMap")
        .def("__init__",
             [](ClassMap* self, const py::dict& entries) {
                 new (self) ClassMap();
                 for (auto item : entries) {
                     int key = py::cast<int>(item.first);
                     std::string value = py::cast<std::string>(item.second);
                     self->class_map.emplace(key, value);
                 }
             })
        .def(
            "get",
            [](const ClassMap& self, int key, const py::object& default_value) {
                auto it = self.class_map.find(key);
                if (it == self.class_map.end()) {
                    return py::cast(default_value);
                }
                return py::cast(it->second);
            },
            py::arg("key"), py::arg("default_value") = py::none())
        .def("__getitem__", [](const ClassMap& self, int key) { return self.class_map.at(key); })
        .def("__eq__",
             [](const ClassMap& left, const py::object& right) {
                 if (!py::isinstance<ClassMap>(right)) {
                     return false;
                 }
                 return left == py::cast<ClassMap>(right);
             })
        .def("__ne__", [](const ClassMap& left, const py::object& right) {
            if (!py::isinstance<ClassMap>(right)) {
                return true;
            }
            return left != py::cast<ClassMap>(right);
        });
    py::class_<ouster::sdk::core::ClassMapSet>(module, "ClassMapSet")
        .def("__init__",
             [](ClassMapSet* self, const py::dict& entries) {
                 new (self) ClassMapSet();
                 for (auto item : entries) {
                     std::string key = py::cast<std::string>(item.first);
                     auto value = py::cast<ClassMap>(item.second);
                     self->class_maps.emplace(key, value);
                 }
             })
        .def("keys", &ClassMapSet::keys, R"(
        Get the keys of the class maps in the set.
        Returns:
            A list of strings representing the keys of the class maps in the set.
        )")
        .def(
            "get",
            [](const ClassMapSet& self, const std::string& key, const py::object& default_value) {
                auto it = self.class_maps.find(key);
                if (it == self.class_maps.end()) {
                    return py::cast(default_value);
                }
                return py::cast(it->second);
            },
            py::arg("key"), py::arg("default_value") = py::none())
        .def("__getitem__", [](const ClassMapSet& self,
                               const std::string& key) { return self.class_maps.at(key); })
        .def("__eq__",
             [](const ClassMapSet& left, const py::object& right) {
                 if (!py::isinstance<ClassMapSet>(right)) {
                     return false;
                 }
                 return left == py::cast<ClassMapSet>(right);
             })
        .def("__ne__", [](const ClassMapSet& left, const py::object& right) {
            if (!py::isinstance<ClassMapSet>(right)) {
                return true;
            }
            return left != py::cast<ClassMapSet>(right);
        });

    py::class_<ouster::sdk::core::FrameSetSourceMetadataSet>(module, "FrameSetSourceMetadataSet")
        .def(py::init<>())
        .def("__setitem__",
             [](ouster::sdk::core::FrameSetSourceMetadataSet& self, const std::string& key,
                const py::object& value) {
                 if (py::isinstance<py::str>(value)) {
                     self.entries.emplace(key, py::cast<std::string>(value));
                 } else if (py::isinstance<ClassMapSet>(value)) {
                     self.entries.emplace(key, py::cast<ClassMapSet>(value));
                 } else {
                     throw std::invalid_argument("Value must be either a string or ClassMapSet.");
                 }
             })
        .def("__getitem__",
             [](const ouster::sdk::core::FrameSetSourceMetadataSet& self, const std::string& key) {
                 auto it = self.entries.find(key);
                 if (it == self.entries.end()) {
                     throw py::key_error(("Key not found: " + key).c_str());
                 }
                 const auto& value = it->second;
                 if (value.is<std::string>()) {
                     return py::cast(value.as<std::string>());
                 } else if (value.is<ClassMapSet>()) {
                     return py::cast(value.as<ClassMapSet>());
                 } else {
                     throw std::runtime_error("Metadata value for key " + key +
                                              " is of unsupported type.");
                 }
             })
        .def("keys", &ouster::sdk::core::FrameSetSourceMetadataSet::keys,
             R"(
        Get the keys of the metadata entries in the collection.
        Returns:
            A list of strings representing the keys of the metadata entries.
        )");

    class FrameSetSourceHack : public ouster::sdk::core::AnyFrameSetSource {
       public:
        using ouster::sdk::core::FrameSetSource::close;
    };

    py::class_<ouster::sdk::core::FrameSetSource, PyFrameSetSource>(
        module, "FrameSetSource",
        "FrameSetSource is a base class for reading point cloud data from "
        "various "
        "sources. ")
        .def(py::init<>())
        .def_prop_ro(
            "sensor_info",
            [](const ouster::sdk::core::FrameSetSource& self) { return self.sensor_info(); },
            R"(
        Retrieve sensor information for all sensors in the frame set source.

        Returns:
            A list of `SensorInfo` objects, each containing metadata
            about a sensor, such as serial number, firmware version,
            and calibration details.
        )")
        .def_prop_ro(
            "is_live", [](const ouster::sdk::core::FrameSetSource& self) { return self.is_live(); },
            R"(
        Check if the frame set source is live.

        A live frame set source indicates that it is actively receiving data from a sensor.

        Returns:
            bool: True if the frame set source is live, False otherwise.
        )")
        .def_prop_ro(
            "is_indexed",
            [](const ouster::sdk::core::FrameSetSource& self) { return self.is_indexed(); })
        .def_prop_ro("is_collated", &ouster::sdk::core::FrameSetSource::is_collated)
        .def_prop_ro("contains_collations", &ouster::sdk::core::FrameSetSource::contains_collations)
        .def_prop_ro(
            "frames_num",
            [](const ouster::sdk::core::FrameSetSource& self) { return self.frames_num(); })
        .def_prop_ro("scans_num",
                     [](const ouster::sdk::core::FrameSetSource& self) {
                         PyErr_WarnEx(PyExc_FutureWarning,
                                      "scans_num is deprecated, use frames_num instead.", 2);
                         return self.frames_num();
                     })
        .def("__enter__", [](py::object self) -> py::object { return self; })
        .def(
            "__exit__",
            [](ouster::sdk::core::FrameSetSource* self, py::object& /*exc_type*/,
               py::object& /*exc_value*/,
               py::object& /*traceback*/) { static_cast<FrameSetSourceHack*>(self)->close(); },
            py::arg("exc_type").none(), py::arg("exc_value").none(), py::arg("traceback").none())
        .def("close",
             [](ouster::sdk::core::FrameSetSource* self) {
                 static_cast<FrameSetSourceHack*>(self)->close();
             })
        .def("__length_hint__",
             [](const ouster::sdk::core::FrameSetSource& self) { return self.size_hint(); })
        .def("__getitem__",
             [](const ouster::sdk::core::FrameSetSource& self, int index) { return self[index]; })
        .def(
            "__getitem__",
            [](const ouster::sdk::core::FrameSetSource& self, const py::slice& slice) {
                auto [start, stop, step, slice_length] = slice.compute(self.end() - self.begin());
                auto slicer = new ouster::sdk::core::Slicer(
                    self, static_cast<int>(start), static_cast<int>(stop), static_cast<int>(step));
                return std::shared_ptr<ouster::sdk::core::Slicer>(slicer);
            },
            py::keep_alive<0, 1>())
        .def(
            "slice",
            [](const ouster::sdk::core::FrameSetSource& self, const py::slice& slice) {
                auto [start, stop, step, slice_length] = slice.compute(self.end() - self.begin());
                auto slicer = new ouster::sdk::core::Slicer(
                    self, static_cast<int>(start), static_cast<int>(stop), static_cast<int>(step));
                return std::shared_ptr<ouster::sdk::core::Slicer>(slicer);
            },
            py::keep_alive<0, 1>())
        .def(
            "single",
            [](const ouster::sdk::core::FrameSetSource& self, int index) {
                auto singler = new ouster::sdk::core::Singler(self, index);
                return std::shared_ptr<ouster::sdk::core::Singler>(singler);
            },
            py::keep_alive<0, 1>())
        .def("__len__",
             [](const ouster::sdk::core::FrameSetSource& source) {
                 try {
                     return source.size();
                 } catch (std::exception& e) {
                     // need to rethrow as type error for python to work
                     // correctly
                     throw py::type_error(
                         "Cannot get the length of an unindexed frame set "
                         "source.");
                 }
             })
        .def_prop_ro("full_index",
                     [](const ouster::sdk::core::FrameSetSource* self) {
                         auto& data = self->full_index();
                         size_t shape[2] = {data.size(), 2};

                         auto arr = py::ndarray<py::numpy, const uint64_t>(data.data(), 2, shape,
                                                                           py::cast(self));

                         return arr;
                     })
        .def_prop_ro("individual_index",
                     [](const ouster::sdk::core::FrameSetSource* self) {
                         std::vector<py::ndarray<py::numpy, const uint64_t>> out;
                         auto& data = self->individual_index();
                         for (const auto& index : data) {
                             size_t shape[2] = {index.size(), 2};
                             auto arr = py::ndarray<py::numpy, const uint64_t>(
                                 index.data(), 2, shape, py::cast(self));
                             out.push_back(arr);
                         }
                         return out;
                     })
        .def(
            "single_iter",
            [](const ouster::sdk::core::FrameSetSource& source, int sensor_idx) {
                iterator_holder<ouster::sdk::core::FrameSetIterator> holder{
                    source.begin(sensor_idx), source.end()};
                return holder;
            },
            py::keep_alive<0, 1>() /* Essential: keep object alive while iterator exists */)
        .def(
            "__iter__",
            [](const ouster::sdk::core::FrameSetSource& source) {
                iterator_holder<ouster::sdk::core::FrameSetIterator> holder{source.begin(),
                                                                            source.end()};
                return holder;
            },
            py::keep_alive<0, 1>(), /* Essential: keep object alive while
                                      iterator exists */
            py::sig("def __iter__(self, /) -> Iterator[FrameSet]"))
        .def("metadata_keys", &ouster::sdk::core::FrameSetSource::metadata_keys,
             R"(
        Get the keys of the metadata entries associated with the frame set source.
        Returns:
            A list of strings representing the keys of the metadata entries.
        )")
        .def(
            "metadata",
            [](const ouster::sdk::core::FrameSetSource& self, const std::string& key) {
                const auto& value = self.metadata(key);
                if (value.is<std::string>()) {
                    return py::cast(value.as<std::string>());
                } else if (value.is<ClassMapSet>()) {
                    return py::cast(value.as<ClassMapSet>());
                } else {
                    throw std::runtime_error("Metadata value is of unsupported type.");
                }
            },
            R"(
        Retrieve a specific metadata entry associated with the frame set source.
        Args:
            key (str): The key of the metadata entry to retrieve.
        Returns:
            The `FrameSetSourceMetadata` object corresponding to the provided key.
        )",
            py::sig("def metadata(self, key: str) -> Union[str, ClassMapSet]"));

    py::class_<SensorFrameSetSource, ouster::sdk::core::FrameSetSource>(module,
                                                                        "SensorFrameSetSource")
        .def(py::init<std::string>(), py::arg("file"))
        .def(
            "__init__",
            [](SensorFrameSetSource* self, const std::vector<Sensor>& sensors, double timeout,
               unsigned int queue_size, bool soft_id_check) {
                new (self) SensorFrameSetSource(sensors, timeout, queue_size, soft_id_check);
            },
            py::arg("sensors"), py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
            py::arg("queue_size") = 2, py::arg("soft_id_check") = false)
        .def(
            "__init__",
            [](SensorFrameSetSource* self, const std::string& file, const py::kwargs& kwargs) {
                ouster::sdk::FrameSetSourceOptions opts;
                parse_frame_set_source_options(kwargs, opts);
                new (self) SensorFrameSetSource(file, opts);
            },
            py::arg("file"), py::arg("kwargs"))
        .def(
            "__init__",
            [](SensorFrameSetSource* self, const std::vector<std::string>& file,
               const py::kwargs& kwargs) {
                ouster::sdk::FrameSetSourceOptions opts;
                parse_frame_set_source_options(kwargs, opts);
                new (self) SensorFrameSetSource(file, opts);
            },
            py::arg("file"), py::arg("kwargs"))
        .def(
            "__init__",
            [](SensorFrameSetSource* self, const std::vector<Sensor>& sensors,
               const std::vector<SensorInfo>& metadata, double timeout, unsigned int queue_size,
               bool soft_id_check) {
                new (self)
                    SensorFrameSetSource(sensors, metadata, timeout, queue_size, soft_id_check);
            },
            py::arg("sensors"), py::arg("metadata"),
            py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
            py::arg("queue_size") = 2, py::arg("soft_id_check") = false)
        .def(
            "__init__",
            [](SensorFrameSetSource* self, const std::vector<Sensor>& sensors,
               const std::vector<SensorInfo>& metadata,
               const std::vector<std::vector<FieldType>>& field_types, double timeout,
               unsigned int queue_size, bool soft_id_check) {
                new (self) SensorFrameSetSource(sensors, metadata, field_types, timeout, queue_size,
                                                soft_id_check);
            },
            py::arg("sensors"), py::arg("metadata"), py::arg("fields"),
            py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
            py::arg("queue_size") = 2, py::arg("soft_id_check") = false)
        .def("flush", &SensorFrameSetSource::flush)
        .def("sockets", &SensorFrameSetSource::sockets)
        .def_prop_ro("dropped_frames", &SensorFrameSetSource::dropped_frames)
        .def_prop_ro("dropped_scans",
                     [](SensorFrameSetSource& self) {
                         PyErr_WarnEx(PyExc_FutureWarning,
                                      "dropped_scans is deprecated, use dropped_frames "
                                      "instead.",
                                      1);
                         return self.dropped_frames();
                     })
        .def_prop_ro("id_error_count", &SensorFrameSetSource::id_error_count)
        .def(
            "get_frame",
            [](SensorFrameSetSource& self, double timeout) {
                py::gil_scoped_release release;
                auto packet = self.get_frame(timeout);
                return packet;
            },
            py::arg("timeout") = 0.1)
        .def(
            "get_scan",
            [](SensorFrameSetSource& self, double timeout) {
                PyErr_WarnEx(PyExc_FutureWarning, "get_scan is deprecated, use get_frame instead.",
                             1);
                py::gil_scoped_release release;
                return self.get_frame(timeout);
            },
            py::arg("timeout") = 0.1);

    module.def(
        "open_source",
        [](const std::string& file, bool collate, int sensor_idx, const py::kwargs& kwargs) {
            FrameSetSourceOptions opts;
            parse_frame_set_source_options(kwargs, opts);
            return ouster::sdk::open_source(
                       file, [&](auto& options) { options = opts; }, collate, sensor_idx)
                .child();
        },
        py::arg("source"), py::arg("collate") = true, py::arg("sensor_idx") = -1,
        py::arg("kwargs"));

    module.def(
        "open_source",
        [](const std::vector<std::string>& files, bool collate, int sensor_idx,
           const py::kwargs& kwargs) {
            FrameSetSourceOptions opts;
            parse_frame_set_source_options(kwargs, opts);
            return ouster::sdk::open_source(
                       files, [&](auto& options) { options = opts; }, collate, sensor_idx)
                .child();
        },
        py::arg("source"), py::arg("collate") = true, py::arg("sensor_idx") = -1,
        py::arg("kwargs"));

    module.def(
        "open_packet_source",
        [](const std::string& file, const py::kwargs& kwargs) {
            PacketSourceOptions opts;
            parse_packet_source_options(kwargs, opts);
            return ouster::sdk::open_packet_source(file, [&](auto& options) { options = opts; })
                .child();
        },
        py::arg("source"), py::arg("kwargs"));

    module.def(
        "open_packet_source",
        [](const std::vector<std::string>& files, const py::kwargs& kwargs) {
            PacketSourceOptions opts;
            parse_packet_source_options(kwargs, opts);
            return ouster::sdk::open_packet_source(files, [&](auto& options) { options = opts; })
                .child();
        },
        py::arg("source"), py::arg("kwargs"));

    py::class_<ouster::sdk::core::Collator, ouster::sdk::core::FrameSetSource>(module, "Collator")
        .def("collation_period", &ouster::sdk::core::Collator::collation_period)
        .def("collation_latencies", &ouster::sdk::core::Collator::collation_latencies);

    py::class_<MultiWrapper, ouster::sdk::core::FrameSetSource>(module, "MultiFrameSetSource")
        .def(
            "__init__",
            [](MultiWrapper* self, const py::object& objs) {
                auto sources =
                    py::cast<std::vector<std::shared_ptr<ouster::sdk::core::FrameSetSource>>>(objs);
                new (self) MultiWrapper(sources);
                holders[self] = objs;
            },
            py::arg("sources"));

    module.def(
        "collate",
        [](ouster::sdk::core::FrameSetSource& src, int time_delta) {
            return ouster::sdk::core::Collator(src, time_delta);
        },
        py::arg("source"), py::arg("dt") = 0, py::keep_alive<0, 1>(),
        R"(
    Collate frames from a frame set source.

    This function creates a `Collator` object that combines frames from a frame set source.

    Args:
        source (FrameSetSource): The frame set source to collate.
        dt (int): The time delta in nanoseconds for collating frames. Default is 0 to autodetect.

    Returns:
        Collator: A collator object for the given frame set source.
    )");

    py::class_<ouster::sdk::core::Singler, ouster::sdk::core::FrameSetSource>(module, "Singler",
                                                                              R"(
        Singler is a class for extracting frames from a single sensor.
        )");

    py::class_<ouster::sdk::core::Slicer, ouster::sdk::core::FrameSetSource>(module, "Slicer",
                                                                             R"(
                Slicer is a class for slicing frames from a frame set source.
            )");
}
