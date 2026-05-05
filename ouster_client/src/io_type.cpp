/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include <ouster/compat_ops.h>
#include <ouster/io_type.h>

#include <cstring>
#include <nonstd/optional.hpp>
#include <stdexcept>
#include <string>

namespace ouster {
namespace sdk {
namespace core {

nonstd::optional<std::string> extension_from_io_type(IoType type) {
    if (type == IoType::OSF) {
        return ".osf";
    }
    if (type == IoType::PCAP) {
        return ".pcap";
    }
    if (type == IoType::BAG) {
        return ".bag";
    }
    if (type == IoType::CSV) {
        return ".csv";
    }
    if (type == IoType::PLY) {
        return ".ply";
    }
    if (type == IoType::PCD) {
        return ".pcd";
    }
    if (type == IoType::LAS) {
        return ".las";
    }
    if (type == IoType::MCAP) {
        return ".mcap";
    }
    if (type == IoType::PNG) {
        return ".png";
    }
    if (type == IoType::STL) {
        return ".stl";
    }

    return nonstd::optional<std::string>();
}

std::string to_string(IoType type) {
    switch (type) {
        case IoType::OSF:
            return "OSF";
        case IoType::PCAP:
            return "PCAP";
        case IoType::BAG:
            return "BAG";
        case IoType::PLY:
            return "PLY";
        case IoType::PCD:
            return "PCD";
        case IoType::LAS:
            return "LAS";
        case IoType::MCAP:
            return "MCAP";
        case IoType::PNG:
            return "PNG";
        case IoType::STL:
            return "STL";
        case IoType::SENSOR:
            return "sensor";
        case IoType::CSV:
            return "CSV";
        default:
            throw std::invalid_argument("Unhandled IoType");
    }
}

IoType io_type_from_extension(const std::string& filename) {
    auto idx = filename.find_last_of('.');
    if (idx != std::string::npos) {
        std::string ext = filename.substr(idx + 1);
        if (ext == "osf") {
            return IoType::OSF;
        }
        if (ext == "pcap") {
            return IoType::PCAP;
        }
        if (ext == "bag") {
            return IoType::BAG;
        }
        if (ext == "csv") {
            return IoType::CSV;
        }
        if (ext == "ply") {
            return IoType::PLY;
        }
        if (ext == "pcd") {
            return IoType::PCD;
        }
        if (ext == "las") {
            return IoType::LAS;
        }
        if (ext == "mcap") {
            return IoType::MCAP;
        }
        if (ext == "png") {
            return IoType::PNG;
        }
        if (ext == "stl") {
            return IoType::STL;
        }
    }
    throw std::invalid_argument(
        "Could not detect IO type from file extension. Expecting one of .osf, "
        ".pcap, .bag, .mcap, .csv, .png, .ply, .pcd, .stl or "
        ".las.");
}

IoType io_type(const std::string& source) {
    auto is_dir = is_directory(source);
    if (is_dir.has_value()) {
        if (is_dir.value()) {
            // directory, could be a ros bag
            auto idx = source.find_last_of('.');
            if (idx != std::string::npos) {
                std::string ext = source.substr(idx + 1);
                if (ext == "bag") {
                    return IoType::BAG;
                }
            }

            throw std::invalid_argument(
                "Source type of '" + source +
                "' not found. Was a directory and was not a ROSbag.");
        } else {
            // file, check the extension
            return io_type_from_extension(source);
        }
    } else {
        // not a file, check if its a sensor
        if (is_host(source)) {
            return IoType::SENSOR;
        }
    }

    throw std::invalid_argument("Source type of '" + source +
                                "' not found. File or host not found.");
}
}  // namespace core
}  // namespace sdk
}  // namespace ouster
