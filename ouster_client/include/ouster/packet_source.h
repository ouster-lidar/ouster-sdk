/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "ouster/impl/iterator_base.h"
#include "ouster/lidar_scan.h"
#include "ouster/scan_source.h"
#include "ouster/types.h"

namespace ouster {
namespace sdk {
namespace core {

class PacketSource;
/// Iterator type for PacketSources
using PacketIterator = ouster::sdk::core::impl::BaseIterator<
    std::pair<int, std::shared_ptr<Packet>>, PacketSource>;
/// Iterator implementation type for PacketSources
using PacketIteratorImpl = ouster::sdk::core::impl::BaseIteratorImpl<
    std::pair<int, std::shared_ptr<Packet>>>;

/// Provides a base API for classes that provide access to packets
class OUSTER_API_CLASS PacketSource {
   public:
    OUSTER_API_FUNCTION
    virtual ~PacketSource() = default;

    /// Get begin iterator for container.
    /// Provides each scan from all sensors in time order
    /// @return iterator to first item in source
    OUSTER_API_FUNCTION
    virtual PacketIterator begin() const = 0;

    /// Get end iterator for source.
    /// @return iterator to end of source
    OUSTER_API_FUNCTION
    virtual PacketIterator end() const;

    /// Indicates if the source is streaming from a device, such as a sensor
    /// @return if live or not
    OUSTER_API_FUNCTION
    virtual bool is_live() const = 0;

    /// Get the SensorInfo for each sensor in the source
    /// @return info about each sensor
    OUSTER_API_FUNCTION
    virtual const std::vector<std::shared_ptr<SensorInfo>>& sensor_info()
        const = 0;

   protected:
    OUSTER_API_FUNCTION
    virtual void close() = 0;
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
