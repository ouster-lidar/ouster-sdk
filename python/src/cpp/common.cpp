#include "common.h"

#include <nanobind/stl/function.h>

void parse_packet_source_options(const py::kwargs& args,
                                 ouster::sdk::PacketSourceOptions& options) {
    for (const auto& item : args) {
        // py::handle keys must be cast to check type/value
        if (!py::isinstance<py::str>(item.first)) {
            throw std::invalid_argument("Incorrect key type for kwargs.");
        }

        auto key = py::cast<std::string>(item.first);

        if (key == "lidar_port") {
            if (item.second.is_none()) {
                continue;
            }
            options.lidar_port = nonstd::optional<uint16_t>(py::cast<uint16_t>(item.second));
        } else if (key == "imu_port") {
            if (item.second.is_none()) {
                continue;
            }
            options.imu_port = nonstd::optional<uint16_t>(py::cast<uint16_t>(item.second));
        } else if (key == "no_auto_udp_dest") {
            options.no_auto_udp_dest = py::cast<bool>(item.second);
        } else if (key == "do_not_reinitialize") {
            options.do_not_reinitialize = py::cast<bool>(item.second);
        } else if (key == "timeout") {
            options.timeout = py::cast<float>(item.second);
        } else if (key == "sensor_info") {
            options.sensor_info = py::cast<std::vector<ouster::sdk::core::SensorInfo>>(item.second);
        } else if (key == "sensor_config") {
            options.sensor_config =
                py::cast<std::vector<ouster::sdk::core::SensorConfig>>(item.second);
        } else if (key == "extrinsics") {
            // OPTIMIZATION: Nanobind handles the iteration, shape checking
            // (4x4), and row-major copying automatically via the caster.
            options.extrinsics =
                py::cast<std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>>(item.second);
        } else if (key == "extrinsics_file") {
            options.extrinsics_file = py::cast<std::string>(item.second);
        } else if (key == "index") {
            options.index = py::cast<bool>(item.second);
        } else if (key == "soft_id_check") {
            options.soft_id_check = py::cast<bool>(item.second);
        } else if (key == "config_timeout") {
            options.config_timeout = py::cast<float>(item.second);
        } else if (key == "reuse_ports") {
            options.reuse_ports = py::cast<bool>(item.second);
        } else if (key == "buffer_time_sec") {
            options.buffer_time_sec = py::cast<float>(item.second);
        } else if (key == "meta") {
            options.meta = py::cast<std::vector<std::string>>(item.second);
        } else if (key == "rcvbuf_size") {
            options.rcvbuf_size = py::cast<uint32_t>(item.second);
        } else {
            throw std::invalid_argument("Unknown parameter '" + key + "' for source.");
        }
    }
}

void parse_frame_set_source_options(const py::kwargs& args,
                                    ouster::sdk::FrameSetSourceOptions& options) {
    for (const auto& item : args) {
        if (!py::isinstance<py::str>(item.first)) {
            throw std::invalid_argument("Incorrect key type for kwargs.");
        }

        auto key = py::cast<std::string>(item.first);
        try {
            if (key == "error_handler") {
                if (item.second.is_none()) {
                    continue;
                }
                options.error_handler = py::cast<ouster::sdk::core::error_handler_t>(item.second);
            } else if (key == "lidar_port") {
                if (item.second.is_none()) {
                    continue;
                }
                options.lidar_port = nonstd::optional<uint16_t>(py::cast<uint16_t>(item.second));
            } else if (key == "imu_port") {
                if (item.second.is_none()) {
                    continue;
                }
                options.imu_port = nonstd::optional<uint16_t>(py::cast<uint16_t>(item.second));
            } else if (key == "field_names") {
                if (item.second.is_none()) {
                    continue;
                }
                options.field_names = py::cast<std::vector<std::string>>(item.second);
            } else if (key == "no_auto_udp_dest") {
                options.no_auto_udp_dest = py::cast<bool>(item.second);
            } else if (key == "reuse_ports") {
                options.reuse_ports = py::cast<bool>(item.second);
            } else if (key == "do_not_reinitialize") {
                options.do_not_reinitialize = py::cast<bool>(item.second);
            } else if (key == "timeout") {
                options.timeout = py::cast<float>(item.second);
            } else if (key == "sensor_info") {
                options.sensor_info =
                    py::cast<std::vector<ouster::sdk::core::SensorInfo>>(item.second);
            } else if (key == "sensor_config") {
                options.sensor_config =
                    py::cast<std::vector<ouster::sdk::core::SensorConfig>>(item.second);
            } else if (key == "extrinsics") {
                // OPTIMIZATION: Nanobind automatically converts
                // list-of-numpy-arrays to std::vector<Eigen::Matrix>.
                options.extrinsics =
                    py::cast<std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>>(
                        item.second);
            } else if (key == "extrinsics_file") {
                options.extrinsics_file = py::cast<std::string>(item.second);
            } else if (key == "index") {
                options.index = py::cast<bool>(item.second);
            } else if (key == "soft_id_check") {
                options.soft_id_check = py::cast<bool>(item.second);
            } else if (key == "config_timeout") {
                options.config_timeout = py::cast<float>(item.second);
            } else if (key == "queue_size") {
                options.queue_size = py::cast<unsigned int>(item.second);
            } else if (key == "raw_headers") {
                options.raw_headers = py::cast<bool>(item.second);
            } else if (key == "raw_fields") {
                options.raw_fields = py::cast<bool>(item.second);
            } else if (key == "meta") {
                options.meta = py::cast<std::vector<std::string>>(item.second);
            } else if (key == "rcvbuf_size") {
                options.rcvbuf_size = py::cast<uint32_t>(item.second);
            } else {
                // throw specific errors for deprecated/removed parameters
                if (key == "cycle") {
                    throw std::invalid_argument(
                        "Parameter 'cycle' is no longer supported. Please "
                        "provide "
                        "on_eof='loop' to SimpleViz constructor or manually "
                        "loop "
                        "instead.");
                } else if (key == "complete") {
                    throw std::invalid_argument(
                        "Parameter 'complete' is no longer supported. Please "
                        "check "
                        "for LidarFrame.complete() explicitly instead.");
                }
                throw std::invalid_argument("Unknown parameter '" + key + "' for source.");
            }
        } catch (py::cast_error& error) {  // nanobind throws cast_error, not
                                           // runtime_error for type mismatches
            throw std::invalid_argument("Invalid type for parameter '" + key + "'.");
        }
    }
}

// -- Helper Constants for DLPack --
// defined in dlpack.h usually, but replicating logic here for clarity if
// headers vary
constexpr uint8_t DL_INT = 0U;
constexpr uint8_t DL_UINT = 1U;
constexpr uint8_t DL_FLOAT = 2U;

// ==============================================================================
// Constructors
// ==============================================================================

OusterDtype::OusterDtype(const ouster::sdk::core::ChanFieldType& ftype)
    : _numeric(false), _string(false), _object(false) {
    from(ftype);
}

OusterDtype::OusterDtype(const py::dlpack::dtype& dtype)
    : _numeric(false), _string(false), _object(false) {
    from(dtype);
}

OusterDtype::OusterDtype(const std::string& stype)
    : _numeric(false), _string(false), _object(false) {
    from(stype);
}

OusterDtype::OusterDtype(const py::object& otype)
    : _numeric(false), _string(false), _object(false) {
    from(otype);
}

// ==============================================================================
// Public Accessors
// ==============================================================================

py::dlpack::dtype OusterDtype::dtype() const {
    py::dlpack::dtype result = {0, 0, 1};  // Default code, bits, lanes
    if (stype_ == "uint8") {
        result.code = DL_UINT;
        result.bits = 8;
    } else if (stype_ == "uint16") {
        result.code = DL_UINT;
        result.bits = 16;
    } else if (stype_ == "uint32") {
        result.code = DL_UINT;
        result.bits = 32;
    } else if (stype_ == "uint64") {
        result.code = DL_UINT;
        result.bits = 64;
    } else if (stype_ == "int8") {
        result.code = DL_INT;
        result.bits = 8;
    } else if (stype_ == "int16") {
        result.code = DL_INT;
        result.bits = 16;
    } else if (stype_ == "int32") {
        result.code = DL_INT;
        result.bits = 32;
    } else if (stype_ == "int64") {
        result.code = DL_INT;
        result.bits = 64;
    } else if (stype_ == "float32") {
        result.code = DL_FLOAT;
        result.bits = 32;
    } else if (stype_ == "float64") {
        result.code = DL_FLOAT;
        result.bits = 64;
    } else if (stype_ == "float16") {
        result.code = DL_FLOAT;
        result.bits = 16;
    } else {
        throw std::runtime_error("OusterDtype: Cannot convert type '" + stype_ +
                                 "' to dlpack dtype.");
    }

    return result;
}

ouster::sdk::core::ChanFieldType OusterDtype::cft() const {
    using namespace ouster::sdk::core;

    if (stype_ == "void") {
        return ChanFieldType::VOID;
    }
    if (stype_ == "uint8") {
        return ChanFieldType::UINT8;
    }
    if (stype_ == "uint16") {
        return ChanFieldType::UINT16;
    }
    if (stype_ == "uint32") {
        return ChanFieldType::UINT32;
    }
    if (stype_ == "uint64") {
        return ChanFieldType::UINT64;
    }
    if (stype_ == "int8") {
        return ChanFieldType::INT8;
    }
    if (stype_ == "int16") {
        return ChanFieldType::INT16;
    }
    if (stype_ == "int32") {
        return ChanFieldType::INT32;
    }
    if (stype_ == "int64") {
        return ChanFieldType::INT64;
    }
    if (stype_ == "float32") {
        return ChanFieldType::FLOAT32;
    }
    if (stype_ == "float64") {
        return ChanFieldType::FLOAT64;
    }
    if (stype_ == "float16") {
        return ChanFieldType::FLOAT16;
    }
    if (stype_ == "char" || stype_ == "str" || stype_ == "S1") {
        return ChanFieldType::CHAR;
    }
    if (stype_ == "zone_state") {
        return ChanFieldType::ZONE_STATE;
    }

    return ChanFieldType::UNREGISTERED;
}

std::string OusterDtype::stype() const {
    return stype_;
}

py::object OusterDtype::otype() const {
    // Converts the internal string to a numpy dtype object
    // Equivalent to python: numpy.dtype(stype_)
    try {
        // static py::object sz_type;// = py::none();
        py::module_ numpy = py::module_::import_("numpy");
        if (stype_ == "zone_state") {
            // Explicitly define offsets to ensure a packed layout
            // (byte-aligned). This prevents NumPy from inserting padding bytes
            // (e.g. after bool) and ensures the itemsize is exactly as expected
            // and matches the save format.
            py::dict dtype_dict;
            py::list names;
            py::list formats;
            py::list offsets;

            // Helper to build the lists
            auto add_field = [&](const char* name, const char* fmt, int offset) {
                names.append(name);
                formats.append(fmt);
                offsets.append(offset);
            };

            add_field("live", "uint8", 0);
            add_field("id", "uint8", 1);
            add_field("error_flags", "uint8", 2);
            add_field("trigger_type", "uint8", 3);
            add_field("trigger_status", "uint8", 4);
            add_field("triggered_frames", "uint32", 5);
            add_field("count", "uint32", 9);
            add_field("occlusion_count", "uint32", 13);
            add_field("invalid_count", "uint32", 17);
            add_field("max_count", "uint32", 21);
            add_field("min_range", "uint32", 25);
            add_field("max_range", "uint32", 29);
            add_field("mean_range", "uint32", 33);

            dtype_dict["names"] = names;
            dtype_dict["formats"] = formats;
            dtype_dict["offsets"] = offsets;
            dtype_dict["itemsize"] = 37;  // Explicitly set packed size

            return numpy.attr("dtype")(dtype_dict);
        }
        return numpy.attr("dtype")(stype_);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("OusterDtype: Failed to create numpy dtype: ") +
                                 e.what());
    }
}

// ==============================================================================
// Protected Helper Methods (from)
// ==============================================================================

void OusterDtype::from(const ouster::sdk::core::ChanFieldType& ftype) {
    using namespace ouster::sdk::core;
    _numeric = true;
    _string = false;
    _object = false;
    switch (ftype) {
        case ChanFieldType::VOID:
            stype_ = "void";
            break;
        case ChanFieldType::UINT8:
            stype_ = "uint8";
            break;
        case ChanFieldType::UINT16:
            stype_ = "uint16";
            break;
        case ChanFieldType::UINT32:
            stype_ = "uint32";
            break;
        case ChanFieldType::UINT64:
            stype_ = "uint64";
            break;
        case ChanFieldType::INT8:
            stype_ = "int8";
            break;
        case ChanFieldType::INT16:
            stype_ = "int16";
            break;
        case ChanFieldType::INT32:
            stype_ = "int32";
            break;
        case ChanFieldType::INT64:
            stype_ = "int64";
            break;
        case ChanFieldType::FLOAT32:
            stype_ = "float32";
            break;
        case ChanFieldType::FLOAT64:
            stype_ = "float64";
            break;
        case ChanFieldType::FLOAT16:
            stype_ = "float16";
            break;
        case ChanFieldType::CHAR:
            _numeric = false;
            _string = true;
            _object = false;
            stype_ = "S1";
            break;
        case ChanFieldType::ZONE_STATE:
            _numeric = false;
            _string = false;
            _object = true;
            stype_ = "zone_state";
            break;
        default:
            _numeric = false;
            _string = false;
            _object = false;
            stype_ = "unregistered";
            break;
    }
}

void OusterDtype::from(const py::dlpack::dtype& dtype) {
    if (dtype.lanes != 1) {
        throw std::runtime_error("OusterDtype: Vectorized types (lanes > 1) are not supported.");
    }

    if (dtype.code == DL_UINT) {
        if (dtype.bits == 8) {
            stype_ = "uint8";
        } else if (dtype.bits == 16) {
            stype_ = "uint16";
        } else if (dtype.bits == 32) {
            stype_ = "uint32";
        } else if (dtype.bits == 64) {
            stype_ = "uint64";
        } else {
            throw std::runtime_error("Unsupported uint bit width");
        }
    } else if (dtype.code == DL_INT) {
        if (dtype.bits == 8) {
            stype_ = "int8";
        } else if (dtype.bits == 16) {
            stype_ = "int16";
        } else if (dtype.bits == 32) {
            stype_ = "int32";
        } else if (dtype.bits == 64) {
            stype_ = "int64";
        } else {
            throw std::runtime_error("Unsupported int bit width");
        }
    } else if (dtype.code == DL_FLOAT) {
        if (dtype.bits == 16) {
            stype_ = "float16";
        } else if (dtype.bits == 32) {
            stype_ = "float32";
        } else if (dtype.bits == 64) {
            stype_ = "float64";
        } else {
            throw std::runtime_error("Unsupported float bit width");
        }
    } else {
        throw std::runtime_error("Unsupported DLPack type code.");
    }

    _numeric = true;
    _string = false;
    _object = false;
}

void OusterDtype::from(const std::string& stype) {
    _numeric = false;
    _string = false;
    _object = false;
    if (stype == "uint8" || stype == "uint16" || stype == "uint32" || stype == "uint64" ||
        stype == "int8" || stype == "int16" || stype == "int32" || stype == "int64" ||
        stype == "float32" || stype == "float64" || stype == "float16") {
        // Got a number
        _numeric = true;
        stype_ = stype;
    } else if (stype == "char" || stype == "str" || stype == "S1") {
        // Got a string
        _string = true;
        stype_ = "S1";  // Normalize to S1
    } else if (stype == "zone_state") {
        // Got an object
        _object = true;
        stype_ = stype;
    } else if (stype == "void") {
        stype_ = stype;
    } else {
        auto bytes_found = stype.find("bytes");
        if (bytes_found != std::string::npos) {
            // Handle "bytesN" types as strings
            _string = true;
            stype_ = "S1";  // Normalize to S1
            return;
        } else {
            throw std::runtime_error("OusterDtype: Unsupported type string '" + stype + "'.");
        }
    }
}

void OusterDtype::from(const py::object& otype) {
    try {
        // Import numpy to act as a universal converter.
        // This handles:
        // 1. Python types: int -> "int64", float -> "float64"
        // 2. Numpy dtypes: np.dtype("uint8") -> "uint8"
        // 3. Shorthand strings: "i4" -> "int32"
        py::object numpy = py::module_::import_("numpy");

        // Create a numpy dtype object from the input
        py::object dtype = numpy.attr("dtype")(otype);

        // Extract the canonical name (e.g. "uint8", "float32") and
        // Delegate to the string implementation
        from(py::cast<std::string>(dtype.attr("name")));
    } catch (const std::exception& e) {
        throw std::runtime_error(
            std::string("OusterDtype: Failed to convert object to type string: ") + e.what());
    }
}

bool OusterDtype::is_numeric() const {
    return _numeric;
}

bool OusterDtype::is_string() const {
    return _string;
}

bool OusterDtype::is_object() const {
    return _object;
}
