/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <vector>

#include "nonstd/optional.hpp"
#include "ouster/io_type.h"

namespace ouster {

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
    friend struct ouster::ScanSourceOptions;
    friend struct ouster::PacketSourceOptions;
    bool set_ = false;
    bool ever_set_ = false;
    T value_;

    Parameter() {}

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

typedef std::function<ouster::core::ScanSource*(
    const std::vector<std::string>& s, const ouster::ScanSourceOptions&)>
    ScanBuilderT;

typedef std::function<ouster::core::PacketSource*(
    const std::vector<std::string>& s, const ouster::PacketSourceOptions&)>
    PacketBuilderT;

OUSTER_API_FUNCTION
std::map<core::IoType, ScanBuilderT>& get_builders();

OUSTER_API_FUNCTION
std::map<core::IoType, PacketBuilderT>& get_packet_builders();

template <core::IoType type, class T>
class ScanSourceBuilder {
   public:
    virtual ~ScanSourceBuilder() { (void)registered_; }

    static bool register_type() {
        get_builders()[type] = [](const std::vector<std::string>& sources,
                                  const ouster::ScanSourceOptions& options) {
            if (sources.size() > 1) {
                throw std::invalid_argument(
                    "This scan source type only allows opening one file/stream "
                    "at a time.");
            }
            return new T(sources[0], options);
        };
        return true;
    }

    static const bool registered_;
};

template <core::IoType type, class T>
class ScanSourceBuilderMulti {
   public:
    virtual ~ScanSourceBuilderMulti() { (void)registered_; }

    static bool register_type() {
        get_builders()[type] = [](const std::vector<std::string>& sources,
                                  const ouster::ScanSourceOptions& options) {
            return new T(sources, options);
        };
        return true;
    }

    static const bool registered_;
};

template <core::IoType type, typename T>
const bool ScanSourceBuilder<type, T>::registered_ =
    ScanSourceBuilder<type, T>::register_type();

template <core::IoType type, typename T>
const bool ScanSourceBuilderMulti<type, T>::registered_ =
    ScanSourceBuilderMulti<type, T>::register_type();

template <core::IoType type, class T>
class PacketSourceBuilder {
   public:
    virtual ~PacketSourceBuilder() { (void)registered_; }

    static bool register_type() {
        get_packet_builders()[type] =
            [](const std::vector<std::string>& sources,
               const ouster::PacketSourceOptions& options) {
                if (sources.size() > 1) {
                    throw std::invalid_argument(
                        "This scan source type only allows opening one "
                        "file/stream at a time.");
                }
                return new T(sources[0], options);
            };
        return true;
    }

    static const bool registered_;
};

template <core::IoType type, class T>
class PacketSourceBuilderMulti {
   public:
    virtual ~PacketSourceBuilderMulti() { (void)registered_; }

    static bool register_type() {
        get_packet_builders()[type] =
            [](const std::vector<std::string>& sources,
               const ouster::PacketSourceOptions& options) {
                return new T(sources, options);
            };
        return true;
    }

    static const bool registered_;
};

template <core::IoType type, typename T>
const bool PacketSourceBuilder<type, T>::registered_ =
    PacketSourceBuilder<type, T>::register_type();

template <core::IoType type, typename T>
const bool PacketSourceBuilderMulti<type, T>::registered_ =
    PacketSourceBuilderMulti<type, T>::register_type();
}  // namespace impl
}  // namespace ouster
