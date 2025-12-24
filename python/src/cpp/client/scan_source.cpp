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
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pyerrors.h>
#include <warnings.h>

#include "common.h"
#include "ouster/defaults.h"
#include "ouster/lidar_scan_set.h"
#include "ouster/osf/osf_scan_source.h"
#include "ouster/packet_source.h"
#include "ouster/pcap_scan_source.h"
#include "ouster/sensor_packet_source.h"
#include "ouster/sensor_scan_source.h"
#include "ouster/types.h"

namespace py = pybind11;
using ouster::sdk::PacketSourceOptions;
using ouster::sdk::ScanSourceOptions;
using ouster::sdk::core::FieldType;
using ouster::sdk::core::LidarScanSet;
using ouster::sdk::core::ScanSource;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::sensor::Sensor;
using ouster::sdk::sensor::SensorPacketSource;
using ouster::sdk::sensor::SensorScanSource;

static std::map<void*, py::object> holders;

class PyPacketSource : public ouster::sdk::core::PacketSource {
    mutable std::vector<std::shared_ptr<SensorInfo>> sensor_info_;

   public:
    PyPacketSource() = default;

    // dummy
    ouster::sdk::core::PacketIterator begin() const override {
        return ouster::sdk::core::PacketIterator(this);
    }

    void close() override {
        PYBIND11_OVERRIDE_PURE(
            void,         /* Return type */
            PacketSource, /* Parent class */
            close         /* Name of function in C++ (must match Python name) */
        );
    }

    bool is_live() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(this);
        auto res = self.attr("is_live");
        return py::cast<bool>(res);
    }

    const std::vector<std::shared_ptr<SensorInfo>>& sensor_info()
        const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(this);
        auto res = self.attr("sensor_info");
        sensor_info_ = py::cast<std::vector<std::shared_ptr<SensorInfo>>>(res);
        return sensor_info_;
    }
};

class PythonScanIteratorImpl : public ouster::sdk::core::ScanIteratorImpl {
    py::iterator iterator_;

    LidarScanSet scans_;
    uint64_t desired_sn_;

   public:
    PythonScanIteratorImpl(py::iterator iter, uint64_t desired_sn = 0)
        : iterator_(iter), desired_sn_(desired_sn) {}

    ~PythonScanIteratorImpl() override {
        py::gil_scoped_acquire acquire;
        iterator_ = {};
    }

    bool advance(size_t offset) override {
        py::gil_scoped_acquire acquire;
        for (size_t i = 0; i < offset; i++) {
            if (iterator_ == py::iterator::sentinel()) {
                return true;
            }
            auto value = *iterator_;
            scans_ = value.cast<LidarScanSet>();
            ++iterator_;

            if (scans_.size() != 1) {
                throw std::runtime_error("Must only yield arrays of 1 scan.");
            }

            // skip data from undesired sensors
            if (desired_sn_ != 0 && scans_[0]->sensor_info->sn != desired_sn_) {
                offset++;
            }
        }
        return false;
    }

    LidarScanSet value() override { return scans_; }
};

class PyScanSource : public ouster::sdk::core::ScanSource {
    mutable std::vector<std::shared_ptr<SensorInfo>> sensor_info_;

   public:
    PyScanSource() = default;

    ouster::sdk::core::ScanIterator begin() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(this);
        auto it = self.attr("__iter__")();
        return ouster::sdk::core::ScanIterator(this,
                                               new PythonScanIteratorImpl(it));
    }

    ouster::sdk::core::ScanIterator begin(int sensor_idx) const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(this);
        auto it = self.attr("__iter__")();
        auto serial_num = sensor_info()[sensor_idx]->sn;
        return ouster::sdk::core::ScanIterator(
            this, new PythonScanIteratorImpl(it, serial_num));
    }

    bool is_live() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(this);
        auto res = self.attr("is_live");
        return py::cast<bool>(res);
    }

    size_t size_hint() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(this);
        auto res = self.attr("__length_hint__")();
        return py::cast<size_t>(res);
    }

    const std::vector<std::shared_ptr<SensorInfo>>& sensor_info()
        const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(this);
        auto res = self.attr("sensor_info");
        sensor_info_ = py::cast<std::vector<std::shared_ptr<SensorInfo>>>(res);
        return sensor_info_;
    }

    // dummy
    std::unique_ptr<ouster::sdk::core::ScanSource> move() override {
        throw std::runtime_error("Moving not supported with this type.");
    }

   protected:
    void close() override {
        PYBIND11_OVERRIDE_PURE(
            void,       /* Return type */
            ScanSource, /* Parent class */
            close       /* Name of function in C++ (must match Python name) */
        );
    }
};

class MultiWrapper : public ouster::sdk::core::MultiScanSource {
   public:
    MultiWrapper(
        const std::vector<std::shared_ptr<ouster::sdk::core::ScanSource>>& srcs)
        : ouster::sdk::core::MultiScanSource(srcs) {}

    ~MultiWrapper() { holders.erase(this); }
};

void init_client_scan_source(py::module& module, py::module& /*unused*/) {
    class PacketSourceHack : public ouster::sdk::core::PacketSource {
       public:
        using ouster::sdk::core::PacketSource::close;
    };
    py::class_<ouster::sdk::core::PacketSource, PyPacketSource,
               std::shared_ptr<ouster::sdk::core::PacketSource>>(module,
                                                                 "PacketSource",
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
        .def_property_readonly("sensor_info",
                               &ouster::sdk::core::PacketSource::sensor_info,
                               R"(
                            Retrieve sensor information for all sensors in the packet source.

                            Returns:
                                A list of `SensorInfo` objects, each containing metadata
                                about a sensor, such as serial number, firmware version,
                                and calibration details.
                            )")
        .def_property_readonly("is_live",
                               &ouster::sdk::core::PacketSource::is_live,
                               R"(
                            Check if the packet source is live.)")
        .def("close",
             [](ouster::sdk::core::PacketSource* self) {
                 ((PacketSourceHack*)self)->close();
             })
        .def("__enter__", [](ouster::sdk::core::PacketSource& /*self*/) {})
        .def("__exit__",
             [](ouster::sdk::core::PacketSource* self,
                pybind11::object& /*exc_type*/, pybind11::object& /*exc_value*/,
                pybind11::object& /*traceback*/) {
                 ((PacketSourceHack*)self)->close();
             })
        .def(
            "__iter__",
            [](const ouster::sdk::core::PacketSource& source) {
                iterator_holder<ouster::sdk::core::PacketIterator> holder{
                    source.begin(), source.end()};
                return holder;
            },
            py::keep_alive<
                0,
                1>() /* Essential: keep object alive while iterator exists */);

    py::class_<iterator_holder<ouster::sdk::core::PacketIterator>>(
        module, "packet_iterator")
        .def("__iter__",
             [](iterator_holder<ouster::sdk::core::PacketIterator>& self)
                 -> iterator_holder<ouster::sdk::core::PacketIterator>& {
                 return self;
             })
        .def("__next__", [](iterator_holder<ouster::sdk::core::PacketIterator>&
                                self) {
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
                std::pair<int, std::shared_ptr<ouster::sdk::core::LidarPacket>>
                    pair;
                pair.first = (*self.iter).first;
                pair.second.reset((ouster::sdk::core::LidarPacket*)copy);
                return py::cast(pair);
            }
            if (ptr->type() == ouster::sdk::core::PacketType::Zone) {
                std::pair<int, std::shared_ptr<ouster::sdk::core::ZonePacket>>
                    pair;
                pair.first = (*self.iter).first;
                pair.second.reset((ouster::sdk::core::ZonePacket*)copy);
                return py::cast(pair);
            } else {
                std::pair<int, std::shared_ptr<ouster::sdk::core::ImuPacket>>
                    pair;
                pair.first = (*self.iter).first;
                pair.second.reset((ouster::sdk::core::ImuPacket*)copy);
                return py::cast(pair);
            }
        });

    py::class_<iterator_holder<ouster::sdk::core::ScanIterator>>(
        module, "scan_iterator")
        .def("__iter__",
             [](iterator_holder<ouster::sdk::core::ScanIterator>& self)
                 -> iterator_holder<ouster::sdk::core::ScanIterator>& {
                 return self;
             })
        .def("__next__",
             [](iterator_holder<ouster::sdk::core::ScanIterator>& self) {
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

    py::class_<SensorPacketSource, ouster::sdk::core::PacketSource,
               std::shared_ptr<SensorPacketSource>>(module,
                                                    "SensorPacketSource",
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
        .def(py::init([](std::vector<Sensor> sensors, double config_timeout,
                         double buffer_time) -> SensorPacketSource* {
                 return new SensorPacketSource(sensors, config_timeout,
                                               buffer_time);
             }),
             py::arg("sensors"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("buffer_time") = 0)
        .def(py::init([](const std::string& file, const py::kwargs& kwargs) {
                 ouster::sdk::PacketSourceOptions opts;
                 parse_packet_source_options(kwargs, opts);
                 return new ouster::sdk::sensor::SensorPacketSource(file, opts);
             }),
             py::arg("file"))
        .def(py::init([](const std::vector<std::string>& file,
                         const py::kwargs& kwargs) {
                 ouster::sdk::PacketSourceOptions opts;
                 parse_packet_source_options(kwargs, opts);
                 return new ouster::sdk::sensor::SensorPacketSource(file, opts);
             }),
             py::arg("file"))
        .def(
            py::init([](std::vector<Sensor> sensors,
                        std::vector<SensorInfo> metadata, double config_timeout,
                        double buffer_size) -> SensorPacketSource* {
                return new SensorPacketSource(sensors, metadata, config_timeout,
                                              buffer_size);
            }),
            py::arg("sensors"), py::arg("metadata"),
            py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
            py::arg("buffer_time") = 0)
        .def("flush", &SensorPacketSource::flush)
        .def("buffer_size", &SensorPacketSource::buffer_size)
        .def(
            "get_packet",
            [](SensorPacketSource& self, double timeout) {
                py::gil_scoped_release release;
                auto packet = self.get_packet(timeout);
                return packet;
            },
            py::arg("timeout") = 0.1)
        .def_property_readonly("dropped_packets",
                               &SensorPacketSource::dropped_packets);

    class ScanSourceHack : public ouster::sdk::core::AnyScanSource {
       public:
        using ouster::sdk::core::ScanSource::close;
    };
    py::class_<ouster::sdk::core::ScanSource, PyScanSource,
               std::shared_ptr<ouster::sdk::core::ScanSource>>(
        module, "ScanSource",
        "ScanSource is a base class for reading point cloud data from various "
        "sources. ")
        .def(py::init<>())
        .def_property_readonly(
            "sensor_info",
            [](const ouster::sdk::core::ScanSource& self) {
                return self.sensor_info();
            },
            R"(
        Retrieve sensor information for all sensors in the scan source.

        Returns:
            A list of `SensorInfo` objects, each containing metadata
            about a sensor, such as serial number, firmware version,
            and calibration details.
        )")
        .def_property_readonly(
            "is_live",
            [](const ouster::sdk::core::ScanSource& self) {
                return self.is_live();
            },
            R"(
        Check if the scan source is live.

        A live scan source indicates that it is actively receiving data from a sensor.

        Returns:
            bool: True if the scan source is live, False otherwise.
        )")
        .def_property_readonly("is_indexed",
                               [](const ouster::sdk::core::ScanSource& self) {
                                   return self.is_indexed();
                               })
        .def_property_readonly("scans_num",
                               [](const ouster::sdk::core::ScanSource& self) {
                                   return self.scans_num();
                               })
        .def("__enter__", [](ouster::sdk::core::ScanSource& /*self*/) {})
        .def("__exit__",
             [](ouster::sdk::core::ScanSource* self,
                pybind11::object& /*exc_type*/, pybind11::object& /*exc_value*/,
                pybind11::object& /*traceback*/) {
                 ((ScanSourceHack*)self)->close();
             })
        .def("close",
             [](ouster::sdk::core::ScanSource* self) {
                 ((ScanSourceHack*)self)->close();
             })
        .def("__length_hint__",
             [](const ouster::sdk::core::ScanSource& self) {
                 return self.size_hint();
             })
        .def("__getitem__", [](const ouster::sdk::core::ScanSource& self,
                               int index) { return self[index]; })
        .def(
            "__getitem__",
            [](const ouster::sdk::core::ScanSource& self, py::slice slice) {
                size_t start;
                size_t stop;
                size_t step;
                size_t slice_length;
                if (!slice.compute(self.end() - self.begin(), &start, &stop,
                                   &step, &slice_length)) {
                    throw py::error_already_set();
                }
                auto slicer =
                    new ouster::sdk::core::Slicer(self, start, stop, step);
                return std::shared_ptr<ouster::sdk::core::Slicer>(slicer);
            },
            py::keep_alive<
                0, 1>() /* Essential: keep object alive while wrapper exists */)
        .def(
            "slice",
            [](const ouster::sdk::core::ScanSource& self, py::slice slice) {
                size_t start;
                size_t stop;
                size_t step;
                size_t slice_length;
                if (!slice.compute(self.end() - self.begin(), &start, &stop,
                                   &step, &slice_length)) {
                    throw py::error_already_set();
                }
                auto slicer =
                    new ouster::sdk::core::Slicer(self, start, stop, step);
                return std::shared_ptr<ouster::sdk::core::Slicer>(slicer);
            },
            py::keep_alive<
                0, 1>() /* Essential: keep object alive while wrapper exists */)
        .def(
            "single",
            [](const ouster::sdk::core::ScanSource& self, int index) {
                auto singler = new ouster::sdk::core::Singler(self, index);
                return std::shared_ptr<ouster::sdk::core::Singler>(singler);
            },
            py::keep_alive<
                0, 1>() /* Essential: keep object alive while wrapper exists */)
        .def("__len__",
             [](const ouster::sdk::core::ScanSource& source) {
                 try {
                     return source.size();
                 } catch (std::exception& e) {
                     // need to rethrow as type error for python to work
                     // correctly
                     throw pybind11::type_error(
                         "Cannot get the length of an unindexed scan source.");
                 }
             })
        .def_property_readonly(
            "full_index",
            [](const ouster::sdk::core::ScanSource& self) {
                auto& data = self.full_index();
                std::vector<size_t> shape = {data.size(), 2};
                auto arr = py::array_t<uint64_t>(shape, (uint64_t*)data.data(),
                                                 py::cast(self));
                reinterpret_cast<py::detail::PyArray_Proxy*>(arr.ptr())
                    ->flags &= ~py::detail::npy_api::NPY_ARRAY_WRITEABLE_;
                return arr;
            })
        .def_property_readonly(
            "individual_index",
            [](const ouster::sdk::core::ScanSource& self) {
                std::vector<py::array> out;
                auto& data = self.individual_index();
                for (const auto& index : data) {
                    std::vector<size_t> shape = {index.size(), 2};
                    auto arr = py::array_t<uint64_t>(
                        shape, (uint64_t*)index.data(), py::cast(self));
                    reinterpret_cast<py::detail::PyArray_Proxy*>(arr.ptr())
                        ->flags &= ~py::detail::npy_api::NPY_ARRAY_WRITEABLE_;
                    out.push_back(arr);
                }
                return out;
            })
        .def(
            "single_iter",
            [](const ouster::sdk::core::ScanSource& source, int sensor_idx) {
                iterator_holder<ouster::sdk::core::ScanIterator> holder{
                    source.begin(sensor_idx), source.end()};
                return holder;
            },
            py::keep_alive<
                0,
                1>() /* Essential: keep object alive while iterator exists */)
        .def(
            "__iter__",
            [](const ouster::sdk::core::ScanSource& source) {
                iterator_holder<ouster::sdk::core::ScanIterator> holder{
                    source.begin(), source.end()};
                return holder;
            },
            py::keep_alive<
                0,
                1>() /* Essential: keep object alive while iterator exists */);

    py::class_<SensorScanSource, ouster::sdk::core::ScanSource,
               std::shared_ptr<SensorScanSource>>(module, "SensorScanSource")
        .def(py::init<std::string>(), py::arg("file"))
        .def(py::init([](std::vector<Sensor> sensors, double timeout,
                         unsigned int queue_size,
                         bool soft_id_check) -> SensorScanSource* {
                 return new SensorScanSource(sensors, timeout, queue_size,
                                             soft_id_check);
             }),
             py::arg("sensors"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("queue_size") = 2, py::arg("soft_id_check") = false)
        .def(py::init([](const std::string& file, const py::kwargs& kwargs) {
                 ouster::sdk::ScanSourceOptions opts;
                 parse_scan_source_options(kwargs, opts);
                 return new SensorScanSource(file, opts);
             }),
             py::arg("file"))
        .def(py::init([](const std::vector<std::string>& file,
                         const py::kwargs& kwargs) {
                 ouster::sdk::ScanSourceOptions opts;
                 parse_scan_source_options(kwargs, opts);
                 return new SensorScanSource(file, opts);
             }),
             py::arg("file"))
        .def(py::init([](std::vector<Sensor> sensors,
                         std::vector<SensorInfo> metadata, double timeout,
                         unsigned int queue_size,
                         bool soft_id_check) -> SensorScanSource* {
                 return new SensorScanSource(sensors, metadata, timeout,
                                             queue_size, soft_id_check);
             }),
             py::arg("sensors"), py::arg("metadata"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("queue_size") = 2, py::arg("soft_id_check") = false)
        .def(py::init([](std::vector<Sensor> sensors,
                         std::vector<SensorInfo> metadata,
                         const std::vector<std::vector<FieldType>>& field_types,
                         double timeout, unsigned int queue_size,
                         bool soft_id_check) -> SensorScanSource* {
                 return new SensorScanSource(sensors, metadata, field_types,
                                             timeout, queue_size,
                                             soft_id_check);
             }),
             py::arg("sensors"), py::arg("metadata"), py::arg("fields"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("queue_size") = 2, py::arg("soft_id_check") = false)
        .def("flush", &SensorScanSource::flush)
        .def_property_readonly("dropped_scans",
                               &SensorScanSource::dropped_scans)
        .def_property_readonly("id_error_count",
                               &SensorScanSource::id_error_count)
        .def(
            "get_scan",
            [](SensorScanSource& self, double timeout) {
                py::gil_scoped_release release;
                auto packet = self.get_scan(timeout);
                return packet;
            },
            py::arg("timeout") = 0.1);
    module.def(
        "open_source",
        [](const std::string& file, bool collate, int sensor_idx,
           const py::kwargs& kwargs) {
            ScanSourceOptions opts;
            parse_scan_source_options(kwargs, opts);
            return ouster::sdk::open_source(
                       file, [&](auto& options) { options = opts; }, collate,
                       sensor_idx)
                .child();
        },
        py::arg("source"), py::arg("collate") = true,
        py::arg("sensor_idx") = -1);

    module.def(
        "open_source",
        [](const std::vector<std::string>& files, bool collate, int sensor_idx,
           const py::kwargs& kwargs) {
            ScanSourceOptions opts;
            parse_scan_source_options(kwargs, opts);
            return ouster::sdk::open_source(
                       files, [&](auto& options) { options = opts; }, collate,
                       sensor_idx)
                .child();
        },
        py::arg("source"), py::arg("collate") = true,
        py::arg("sensor_idx") = -1);

    module.def(
        "open_packet_source",
        [](const std::string& file, const py::kwargs& kwargs) {
            PacketSourceOptions opts;
            parse_packet_source_options(kwargs, opts);
            return ouster::sdk::open_packet_source(
                       file, [&](auto& options) { options = opts; })
                .child();
        },
        py::arg("source"));

    module.def(
        "open_packet_source",
        [](const std::vector<std::string>& files, const py::kwargs& kwargs) {
            PacketSourceOptions opts;
            parse_packet_source_options(kwargs, opts);
            return ouster::sdk::open_packet_source(
                       files, [&](auto& options) { options = opts; })
                .child();
        },
        py::arg("source"));

    py::class_<ouster::sdk::core::Collator, ouster::sdk::core::ScanSource,
               std::shared_ptr<ouster::sdk::core::Collator>>(module,
                                                             "Collator");

    py::class_<MultiWrapper, ouster::sdk::core::ScanSource,
               std::shared_ptr<MultiWrapper>>(module, "MultiScanSource")
        .def(py::init([](const py::object& objs) {
                 auto sources = py::cast<std::vector<
                     std::shared_ptr<ouster::sdk::core::ScanSource>>>(objs);
                 auto out = new MultiWrapper(sources);
                 holders[out] = objs;
                 return out;
             }),
             py::arg("sources"));

    module.def(
        "collate",
        [](ouster::sdk::core::ScanSource& src, int time_delta) {
            return ouster::sdk::core::Collator(src, time_delta);
        },
        py::arg("source"), py::arg("dt") = 210000000, py::keep_alive<0, 1>(),
        R"(
    Collate scans from a scan source.

    This function creates a `Collator` object that combines scans from a scan source.

    Args:
        source (ScanSource): The scan source to collate.
        dt (int): The time delta in nanoseconds for collating scans. Default is 210000000.

    Returns:
        Collator: A collator object for the given scan source.
    )");

    py::class_<ouster::sdk::core::Singler, ouster::sdk::core::ScanSource,
               std::shared_ptr<ouster::sdk::core::Singler>>(module, "Singler",
                                                            R"(
        Singler is a class for extracting scans from a single sensor.
        )");

    py::class_<ouster::sdk::core::Slicer, ouster::sdk::core::ScanSource,
               std::shared_ptr<ouster::sdk::core::Slicer>>(module, "Slicer",
                                                           R"(
                Slicer is a class for slicing scans from a scan source.
            )");
}
