/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <memory>
#include <nonstd/optional.hpp>
#include <set>
#include <utility>
#include <vector>

#include "ouster/core/frame_set.h"
#include "ouster/core/generic.h"
#include "ouster/core/impl/iterator_base.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"

namespace ouster {
namespace sdk {
namespace core {

class FrameSetSource;

/// Iterator for FrameSetSources
using FrameSetIterator = ouster::sdk::core::impl::BaseIterator<FrameSet, FrameSetSource>;

/// Iterator implementation for FrameSetSources
using FrameSetIteratorImpl = ouster::sdk::core::impl::BaseIteratorImpl<FrameSet>;

/// Describes a slice into an array-like datastructure
struct OUSTER_API_CLASS Slice {
    int start = 0;                              ///< start index of the slice
    int end = std::numeric_limits<int>::max();  ///< end index of the slice
    int step = 1;                               ///< step size when iterating through the slice
};

// Forward declarations for below
class Slicer;
class Singler;

class OUSTER_API_CLASS FrameSetSourceMetadata : public Generic {
    using Generic::Generic;
};

/**
 * A set of metadata entries for a FrameSetSource, indexed by string keys.
 */
struct OUSTER_API_CLASS FrameSetSourceMetadataSet {
    /// Mapping of metadata keys to metadata entries
    std::unordered_map<std::string, FrameSetSourceMetadata> entries;
    /** Get the set of keys for the metadata entries in this
     * FrameSetSourceMetadataSet.
     * @return set of keys for the metadata entries in this
     * FrameSetSourceMetadataSet
     */
    OUSTER_API_FUNCTION
    std::set<std::string> keys() const;
};

/// Provides a base API for classes that provide access to frames in a file
class OUSTER_API_CLASS FrameSetSource {
   public:
    OUSTER_API_FUNCTION
    virtual ~FrameSetSource() = default;

    /// Provides each frame from all sensors in time order
    /// @return start iterator for all sensors
    OUSTER_API_FUNCTION
    virtual FrameSetIterator begin() const = 0;

    /// Provides frames from a single sensor in time order
    /// If idx < 0 provides frames from all sensors
    /// @param[in] sensor_index sensor index
    /// @return start iterator for the sensor with the given index
    /// @throw std::runtime_error if sensor_idx >= number of sensors
    OUSTER_API_FUNCTION
    virtual FrameSetIterator begin(int sensor_index  ///< [in] sensor index
    ) const = 0;

    /// Return the end iterator
    /// @return end iterator
    OUSTER_API_FUNCTION
    virtual FrameSetIterator end() const;

    /// Get the sensor info for each sensor in this dataset
    /// @return sensor info for each sensor
    OUSTER_API_FUNCTION
    virtual const std::vector<std::shared_ptr<SensorInfo>>& sensor_info() const = 0;

    /// Indicates if the source is streaming from a device, such as a sensor
    /// @return if live or not
    OUSTER_API_FUNCTION
    virtual bool is_live() const;

    /// Indicates if the source contains an index for fast random access
    /// @return if indexed or not
    OUSTER_API_FUNCTION
    virtual bool is_indexed() const;

    /// The approximate length of the source if unindexed, or the real size if
    /// indexed. Live sources or sources with undefined length will return 0.
    /// @return approximate length of source
    OUSTER_API_FUNCTION
    virtual size_t size_hint() const = 0;

    // The below operations are only supported on indexed frame set sources

    /// The length of the source from begin to end.
    /// For example: If uncollated this is the total number of frames in the
    /// source, if collated this is the number of frame collations.
    /// @return length of the source
    /// @throw std::runtime_error if unindexed
    OUSTER_API_FUNCTION
    virtual size_t size() const;

    /// Get the frame count for each sensor in the file
    /// @return frame count for each sensor in the file
    /// @throw std::runtime_error if unindexed
    OUSTER_API_FUNCTION
    virtual const std::vector<size_t>& frames_num() const;

    /// @deprecated Use frames_num() instead
    /// @return frame count for each sensor in the file
    OUSTER_DEPRECATED_MSG("frames_num()", OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    virtual const std::vector<size_t>& scans_num() const;

    /// Returns the Nth frame in time order
    /// Supports negative indices for python-like indexing behavior.
    /// @param[in] index frame index
    /// @return frame at given index
    OUSTER_API_FUNCTION
    virtual FrameSet operator[](int index) const;

    /// timestamp based index of all frames in the file for each sensor, each
    /// pair is timestamp followed by global frame index
    /// @return index
    /// @throw std::runtime_error if unindexed
    OUSTER_API_FUNCTION
    virtual const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>& individual_index() const;

    /// timestamp based index of all frames in the file, each pair is timestamp
    /// followed by sensor index
    /// @return index
    /// @throw std::runtime_error if unindexed
    OUSTER_API_FUNCTION
    virtual const std::vector<std::pair<uint64_t, uint64_t>>& full_index() const;

    // Other useful methods

    /// move into a newly allocated item
    /// @return a unique pointer to a newly allocated `FrameSetSource` instance
    ///         containing the moved-from state.
    OUSTER_API_FUNCTION
    virtual std::unique_ptr<FrameSetSource> move() = 0;

    /// Get only a single sensor's stream from this source
    /// @tparam T hack to allow usage of yet-undefined class. Do not change.
    /// @param[in] sensor_idx Index of the sensor.
    /// @return singled frame set source
    template <class T = Singler>
    T single(int sensor_idx = 0) const& {
        return T(*this, sensor_idx);
    }

    /// Get only a single sensor's stream from this source
    /// @tparam T hack to allow usage of yet-undefined class. Do not change.
    /// @param[in] sensor_idx Index of the sensor.
    /// @return singled frame set source
    template <class T = Singler>
    T single(int sensor_idx = 0) && {
        return T(std::move(*this), sensor_idx);
    }

    /// Slice a frame set source
    /// @tparam T hack to allow usage of yet-undefined class. Do not change.
    /// @param[in] l slice
    /// @return sliced frame set source
    template <class T = Slicer>
    T operator[](std::initializer_list<nonstd::optional<int>> l) const& {
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

    /// @copydoc operator[](std::initializer_list<nonstd::optional<int>>)
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

    /**
     * Check if the frame set source has metadata associated with a given key.
     * @param[in] key The key to check for in the metadata.
     * @return True if the metadata contains the key, false otherwise.
     */
    OUSTER_API_FUNCTION
    virtual bool has_metadata(const std::string& key) const;

    /**
     * Get the keys of the metadata entries associated with the frame set
     * source.
     * @return A set of strings representing the keys of the metadata entries.
     */
    OUSTER_API_FUNCTION
    virtual std::set<std::string> metadata_keys() const;

    /**
     * Get the metadata entry associated with a given key.
     * @param[in] name The key of the metadata entry to retrieve.
     * @return The metadata entry associated with the given key.
     * @throw std::out_of_range if the key is not found in the metadata.
     */
    OUSTER_API_FUNCTION
    virtual const FrameSetSourceMetadata& metadata(const std::string& name) const;

    /**
     * Check if the source provides collated frames.
     *
     * @return true if collations are provided.
     */
    OUSTER_API_FUNCTION
    virtual bool is_collated() const;

    /**
     * Check if the most underlying source stores collated frames.
     *
     * @return true if collations are stored.
     */
    OUSTER_API_FUNCTION
    virtual bool contains_collations() const;

   protected:
    virtual void close() = 0;
    FrameSetSourceMetadataSet metadata_;

    OUSTER_API_FUNCTION
    virtual void add_metadata(const std::string& name, FrameSetSourceMetadata obj);
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster

#include "ouster/core/deprecation.h"

namespace ouster {
namespace sdk {
namespace core {

OUSTER_DEPRECATED_TYPE(ScanSource, FrameSetSource, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(ScanSourceMetadata, FrameSetSourceMetadata,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(ScanSourceMetadataSet, FrameSetSourceMetadataSet,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(ScanIterator, FrameSetIterator, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)

}  // namespace core
}  // namespace sdk
}  // namespace ouster
