/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "ouster/io_type.h"
#include "ouster/scan_source_utils.h"

namespace ouster {
namespace sdk {

// Forward declarations
struct ScanSourceOptions;
struct PacketSourceOptions;
namespace core {
class ScanSource;
class PacketSource;
}  // namespace core

namespace impl {

template <class T>
struct Parameter {
    friend struct ouster::sdk::ScanSourceOptions;
    friend struct ouster::sdk::PacketSourceOptions;
    bool set_ = false;
    bool ever_set_ = false;
    T value_;

    Parameter() = default;

   private:
    Parameter(const T& v) { value_ = v; }
    void check(const char* name, const char* source_name) const {
        if (set_) {
            throw std::runtime_error("Parameter '" + std::string(name) +
                                     "' not supported by " +
                                     std::string(source_name) + ".");
        }
    }

   public:
    T& operator=(const T& b) {
        value_ = b;
        set_ = true;
        ever_set_ = true;
        return value_;
    }

    T& retrieve() {
        set_ = false;
        return value_;
    }

    Parameter<T> move() {
        auto copy = *this;
        set_ = false;
        return copy;
    }

    bool was_set() const { return ever_set_; }
};

template <class T>
T get_scan_options(const std::function<void(T&)>& options) {
    T opts;
    if (options) {
        options(opts);
    }
    return opts;
}

template <class T>
T get_packet_options(const std::function<void(T&)>& options) {
    T opts;
    if (options) {
        options(opts);
    }
    return opts;
}

using ScanBuilderT = std::function<std::unique_ptr<core::ScanSource>(
    const std::vector<std::string>&, const ouster::sdk::ScanSourceOptions&,
    bool /*collate*/, int /*sensor_idx*/)>;

using PacketBuilderT = std::function<std::unique_ptr<core::PacketSource>(
    const std::vector<std::string>&, const ouster::sdk::PacketSourceOptions&)>;

OUSTER_API_FUNCTION
std::map<core::IoType, ScanBuilderT>& get_builders();

OUSTER_API_FUNCTION
std::map<core::IoType, PacketBuilderT>& get_packet_builders();

template <core::IoType type, class T>
class ScanSourceBuilder {
   public:
    virtual ~ScanSourceBuilder() { (void)REGISTERED; }

    static bool register_type() {
        get_builders()[type] = [](const std::vector<std::string>& sources,
                                  const ouster::sdk::ScanSourceOptions& options,
                                  bool collate, int sensor_idx) {
            return T::create(sources, options, collate, sensor_idx);
        };
        return true;
    }

    static const bool REGISTERED;
};

template <core::IoType type, typename T>
const bool ScanSourceBuilder<type, T>::REGISTERED =
    ScanSourceBuilder<type, T>::register_type();

template <core::IoType type, class T>
class PacketSourceBuilder {
   public:
    virtual ~PacketSourceBuilder() { (void)REGISTERED; }

    static bool register_type() {
        get_packet_builders()[type] =
            [](const std::vector<std::string>& sources,
               const ouster::sdk::PacketSourceOptions& options) {
                if (sources.size() > 1) {
                    throw std::invalid_argument(
                        "This scan source type only allows opening one "
                        "file/stream at a time.");
                }
                return std::unique_ptr<core::PacketSource>(
                    new T(sources[0], options));
            };
        return true;
    }

    static const bool REGISTERED;
};

template <core::IoType type, class T>
class PacketSourceBuilderMulti {
   public:
    virtual ~PacketSourceBuilderMulti() { (void)REGISTERED; }

    static bool register_type() {
        get_packet_builders()[type] =
            [](const std::vector<std::string>& sources,
               const ouster::sdk::PacketSourceOptions& options) {
                return std::unique_ptr<core::PacketSource>(
                    new T(sources, options));
            };
        return true;
    }

    static const bool REGISTERED;
};

template <core::IoType type, typename T>
const bool PacketSourceBuilder<type, T>::REGISTERED =
    PacketSourceBuilder<type, T>::register_type();

template <core::IoType type, typename T>
const bool PacketSourceBuilderMulti<type, T>::REGISTERED =
    PacketSourceBuilderMulti<type, T>::register_type();
}  // namespace impl
}  // namespace sdk
}  // namespace ouster
