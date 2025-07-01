/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include "ouster/impl/iterator_base.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

namespace ouster {
namespace core {

class ScanSource;
/// Iterator for ScanSources
typedef ouster::impl::BaseIterator<std::vector<std::shared_ptr<LidarScan>>,
                                   ScanSource>
    ScanIterator;
/// Iterator implementation for ScanSources
typedef ouster::impl::BaseIteratorImpl<std::vector<std::shared_ptr<LidarScan>>>
    ScanIteratorImpl;

/// Describes a slice into an array-like datastructure
struct OUSTER_API_CLASS Slice {
    int start = 0;                              ///< start index of the slice
    int end = std::numeric_limits<int>::max();  ///< end index of the slice
    int step = 1;  ///< step size when iterating through the slice
};

// Forward declarations for below
class Slicer;
class Singler;

/// Provides a base API for classes that provide access to scans in a file
class OUSTER_API_CLASS ScanSource {
   public:
    OUSTER_API_FUNCTION
    virtual ~ScanSource() {}

    /// Provides each scan from all sensors in time order
    /// @return start iterator for all sensors
    OUSTER_API_FUNCTION
    virtual ScanIterator begin() const = 0;

    /// Provides scans from a single sensor in time order
    /// If idx < 0 provides scans from all sensors
    /// @return start iterator for the sensor with the given index
    /// @throw std::runtime_error if sensor_idx >= number of sensors
    OUSTER_API_FUNCTION
    virtual ScanIterator begin(int sensor_index  ///< [in] sensor index
    ) const = 0;

    /// Return the end iterator
    /// @return end iterator
    OUSTER_API_FUNCTION
    virtual ScanIterator end() const;

    /// Get the sensor info for each sensor in this dataset
    /// @return sensor info for each sensor
    OUSTER_API_FUNCTION
    virtual const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
    sensor_info() const = 0;

    /// Indicates if the source is streaming from a device, such as a sensor
    /// @return if live or not
    OUSTER_API_FUNCTION
    virtual bool is_live() const;

    /// Indicates if the source contains an index for fast random access
    /// @return if indexed or not
    OUSTER_API_FUNCTION
    virtual bool is_indexed() const;

    /// timestamp based index of all scans in the file for each sensor, each
    /// pair is timestamp followed by global scan index
    /// @return index
    /// @throw std::runtime_error if unindexed
    OUSTER_API_FUNCTION
    virtual const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
    individual_index() const;

    /// timestamp based index of all scans in the file, each pair is timestamp
    /// followed by sensor index
    /// @return index
    /// @throw std::runtime_error if unindexed
    OUSTER_API_FUNCTION
    virtual const std::vector<std::pair<uint64_t, uint64_t>>& full_index()
        const;

    // The below operations are only supported on indexed scan sources

    /// The length of the source from begin to end.
    /// For example: If uncollated this is the total number of scans in the
    /// source, if collated this is the number of scan collations.
    /// @return length of the source
    /// @throw std::runtime_error if unindexed
    OUSTER_API_FUNCTION
    virtual size_t size() const;

    /// Get the scan count for each sensor in the file
    /// @return scan count for each sensor in the file
    /// @throw std::runtime_error if unindexed
    OUSTER_API_FUNCTION
    virtual const std::vector<size_t>& scans_num() const;

    /// Returns the Nth scan in time order
    /// Supports negative indices for python-like indexing behavior.
    /// @return scan at given index
    OUSTER_API_FUNCTION
    virtual std::vector<std::shared_ptr<LidarScan>> operator[](int index) const;

    // move into a newly allocated item
    OUSTER_API_FUNCTION
    virtual ScanSource* move() = 0;

    // Other useful methods

    /// Get only a single sensor's stream from this source
    /// @tparam T hack to allow usage of yet-undefined class. Do not change.
    /// @return singled scan source
    template <class T = Singler>
    T single(int sensor_idx = 0  /// <[in] index of sensor
    ) const& {
        return T(*this, sensor_idx);
    }

    /// Get only a single sensor's stream from this source
    /// @tparam T hack to allow usage of yet-undefined class. Do not change.
    /// @return singled scan source
    template <class T = Singler>
    T single(int sensor_idx = 0  /// <[in] index of sensor
             ) && {
        return T(std::move(*this), sensor_idx);
    }

    /// Slice a scan source
    /// @tparam T hack to allow usage of yet-undefined class. Do not change.
    /// @return sliced scan source
    template <class T = Slicer>
    T operator[](std::initializer_list<nonstd::optional<int>> l  /// <[in] slice
    ) const& {
        Slice s;
        if (l.size() >= 1 && l.begin()->has_value()) {
            s.start = l.begin()->value();
        }
        if (l.size() >= 2 && (l.begin() + 1)->has_value()) {
            s.end = (l.begin() + 1)->value();
        }
        if (l.size() >= 3 && (l.begin() + 2)->has_value()) {
            s.step = (l.begin() + 2)->value();
        }
        return T(*this, s.start, s.end, s.step);
    }

    /// @copydoc operator[]
    template <class T = Slicer>
    T operator[](std::initializer_list<nonstd::optional<int>> l) && {
        Slice s;
        if (l.size() >= 1 && l.begin()->has_value()) {
            s.start = l.begin()->value();
        }
        if (l.size() >= 2 && (l.begin() + 1)->has_value()) {
            s.end = (l.begin() + 1)->value();
        }
        if (l.size() >= 3 && (l.begin() + 2)->has_value()) {
            s.step = (l.begin() + 2)->value();
        }
        return T(std::move(*this), s.start, s.end, s.step);
    }

   protected:
    virtual void close() = 0;
};

}  // namespace core
}  // namespace ouster
