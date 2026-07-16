/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "ouster/core/frame_set_source.h"
#include "ouster/core/packet_source.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * @brief Base class for FrameSetSource wrappers, which wrap a FrameSetSource
 * and forward all calls to it. This is useful for building FrameSetSources that
 * modify the behavior of an underlying FrameSetSource without having to
 * reimplement all of the FrameSetSource interface.
 */
class OUSTER_API_CLASS FrameSetSourceWrapper : public FrameSetSource {
   protected:
    std::shared_ptr<FrameSetSource> parent_;
    const FrameSetSource* source_;

   protected:
    FrameSetSourceWrapper(const FrameSetSource& source);
    FrameSetSourceWrapper(FrameSetSource&& source);
    FrameSetSourceWrapper(std::unique_ptr<FrameSetSource> source);

    FrameSetSourceWrapper(FrameSetSourceWrapper&&) = default;

    OUSTER_API_FUNCTION void close() override;

   public:
    /// @copydoc FrameSetSource::sensor_info()
    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<SensorInfo>>& sensor_info() const override;

    /// @copydoc FrameSetSource::is_live()
    OUSTER_API_FUNCTION
    bool is_live() const override;

    /// @copydoc FrameSetSource::is_indexed()
    OUSTER_API_FUNCTION
    bool is_indexed() const override;

    /// @copydoc FrameSetSource::is_collated()
    OUSTER_API_FUNCTION
    bool is_collated() const override;

    /// @copydoc FrameSetSource::contains_collations()
    OUSTER_API_FUNCTION
    bool contains_collations() const override;

    /// @copydoc FrameSetSource::size_hint()
    OUSTER_API_FUNCTION
    size_t size_hint() const override;

    /// @copydoc FrameSetSource::full_index()
    OUSTER_API_FUNCTION
    const std::vector<std::pair<uint64_t, uint64_t>>& full_index() const override;

    /// @copydoc FrameSetSource::individual_index()
    OUSTER_API_FUNCTION
    const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>& individual_index()
        const override;

    /// @copydoc FrameSetSource::frames_num()
    OUSTER_API_FUNCTION
    const std::vector<size_t>& frames_num() const override;

    /// @copydoc FrameSetSource::has_metadata()
    OUSTER_API_FUNCTION
    bool has_metadata(const std::string& key) const override;

    /// @copydoc FrameSetSource::metadata_keys()
    OUSTER_API_FUNCTION
    std::set<std::string> metadata_keys() const override;

    /// @copydoc FrameSetSource::metadata()
    OUSTER_API_FUNCTION
    const FrameSetSourceMetadata& metadata(const std::string& name) const override;
};

/// FrameSetSource that collates a given FrameSetSource
class OUSTER_API_CLASS Collator : public FrameSetSourceWrapper {
    friend class DefaultCollatedFrameSetIteratorImpl;
    // parameters for collation
    uint64_t dt_;
    std::vector<int64_t> latencies_;

    // index data
    int64_t length_ = 0;
    std::vector<int32_t> index_;

    void build_index();

   public:
    /// Collate the provided source with the provided cut interval
    OUSTER_API_FUNCTION
    Collator(std::unique_ptr<FrameSetSource> source,  ///< [in] source to collate
             uint64_t dt_ns = 0                       ///< [in] interval in nanoseconds after
                                                      ///< which to cut collated frames 0 = auto
    );

    /// @copydoc Collator::Collator()
    OUSTER_API_FUNCTION
    Collator(const FrameSetSource& source, uint64_t dt_ns = 0);

    /// @copydoc Collator::Collator()
    OUSTER_API_FUNCTION
    Collator(FrameSetSource&& source, uint64_t dt_ns = 0);

    /**
     * Move constructor for Collator.
     * @param[in] copy The Collator instance to move from.
     */
    OUSTER_API_FUNCTION
    Collator(Collator&& copy) = default;

    OUSTER_API_FUNCTION
    FrameSetIterator begin() const override;

    OUSTER_API_FUNCTION
    FrameSetIterator begin(int sensor_index) const override;

    OUSTER_API_FUNCTION
    std::unique_ptr<FrameSetSource> move() override;

    OUSTER_API_FUNCTION
    size_t size_hint() const override;

    OUSTER_API_FUNCTION
    bool is_collated() const override;

    /// Get the provided or autodetected collation period in nanoseconds.
    /// @return the collation period in nanoseconds
    OUSTER_API_FUNCTION
    int64_t collation_period() const;

    /// Get the provided or autodetected collation offsets between frames in ns.
    /// @return the collation period in nanoseconds
    OUSTER_API_FUNCTION
    std::vector<int64_t> collation_latencies() const;
};

/// Build a frame set source that collates the provided source
/// @return frame set source collating the provided source
OUSTER_API_FUNCTION
Collator collate(const FrameSetSource& source);

/// @copydoc collate()
OUSTER_API_FUNCTION
Collator collate(FrameSetSource&& source);

/// FrameSetSource that iterates over a single stream in a FrameSetSource
class OUSTER_API_CLASS Singler : public FrameSetSourceWrapper {
    size_t idx_;
    std::vector<std::shared_ptr<SensorInfo>> sensor_info_;
    std::vector<size_t> frames_num_;

   public:
    /// Single the provided source, getting the sensor with the provided index
    OUSTER_API_FUNCTION
    Singler(const FrameSetSource& source,  ///< [in] source to single
            size_t idx                     ///< [in] index of sensor to single out
    );

    /// @copydoc Singler::Singler()
    OUSTER_API_FUNCTION
    Singler(FrameSetSource&& source, size_t idx);

    /// @copydoc Singler::Singler()
    OUSTER_API_FUNCTION
    Singler(std::unique_ptr<FrameSetSource> source, size_t idx);

    /**
     * Move constructor for Singler.
     * @param[in] copy The Singler instance to move from.
     */
    OUSTER_API_FUNCTION
    Singler(Singler&& copy) = default;

    OUSTER_API_FUNCTION
    FrameSetIterator begin() const override;

    OUSTER_API_FUNCTION
    FrameSetIterator begin(int idx) const override;

    OUSTER_API_FUNCTION
    FrameSetIterator end() const override;

    OUSTER_API_FUNCTION
    const std::vector<size_t>& frames_num() const override;

    OUSTER_API_FUNCTION
    size_t size_hint() const override;

    OUSTER_API_FUNCTION
    bool is_collated() const override;

    OUSTER_API_FUNCTION
    std::unique_ptr<FrameSetSource> move() override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<SensorInfo>>& sensor_info() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::pair<uint64_t, uint64_t>>& full_index() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>& individual_index()
        const override;

   private:
    void build_index();

    std::vector<std::vector<std::pair<uint64_t, uint64_t>>> individual_index_;
    std::vector<std::pair<uint64_t, uint64_t>> full_index_;
};

/// FrameSetSource that slices a given FrameSetSource
class OUSTER_API_CLASS Slicer : public FrameSetSourceWrapper {
    int start_;
    int end_;
    int step_;

   public:
    /// Slice the provided source with the provided bounds
    OUSTER_API_FUNCTION
    Slicer(const FrameSetSource& source,  ///< [in] source to slice
           int start,                     ///< [in] start index of slice
           int end,                       ///< [in] end index of slice
           int step                       ///< [in] step size of slice
    );

    /// @copydoc Slicer::Slicer()
    OUSTER_API_FUNCTION
    Slicer(FrameSetSource&& source, int start, int end, int step);

    /**
     * Move constructor for Slicer.
     * @param[in] source The Slicer instance to move from.
     */
    OUSTER_API_FUNCTION
    Slicer(Slicer&& source) = default;

    OUSTER_API_FUNCTION
    FrameSetIterator begin() const override;

    OUSTER_API_FUNCTION
    FrameSetIterator begin(int idx) const override;

    OUSTER_API_FUNCTION
    std::unique_ptr<FrameSetSource> move() override;

    OUSTER_API_FUNCTION
    size_t size_hint() const override;

    OUSTER_API_FUNCTION
    const std::vector<size_t>& frames_num() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::pair<uint64_t, uint64_t>>& full_index() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>& individual_index()
        const override;

   private:
    void build_index();

    std::vector<std::vector<std::pair<uint64_t, uint64_t>>> individual_index_;
    std::vector<std::pair<uint64_t, uint64_t>> full_index_;
    std::vector<size_t> frames_num_;
};

/// Wraps a shared pointer to a FrameSetSource to allow easy use of it by value
class OUSTER_API_CLASS AnyFrameSetSource : public FrameSetSourceWrapper {
   public:
    /// Construct an AnyFrameSetSource from a provided source, taking ownership
    /// of it
    OUSTER_API_FUNCTION
    AnyFrameSetSource(
        std::unique_ptr<FrameSetSource> source  ///< [in] the source to wrap and take ownership of
    );

    OUSTER_API_FUNCTION
    FrameSetIterator begin() const override;

    OUSTER_API_FUNCTION
    FrameSetIterator begin(int idx) const override;

    OUSTER_API_FUNCTION
    FrameSetIterator end() const override;

    OUSTER_API_FUNCTION
    std::unique_ptr<FrameSetSource> move() override;

    /// Get the FrameSetSource wrapped by this FrameSetSource
    /// @return wrapped FrameSetSource
    OUSTER_API_FUNCTION
    std::shared_ptr<FrameSetSource> child() const;

   protected:
    OUSTER_API_FUNCTION void close() override;
};

/// Wraps a shared pointer to a FrameSetSource to allow easy use of it by value
class OUSTER_API_CLASS AnyPacketSource : public PacketSource {
    std::shared_ptr<PacketSource> source_;

   public:
    /// Construct an AnyPacketSource from a provided source, taking ownership of
    /// it
    OUSTER_API_FUNCTION
    AnyPacketSource(
        std::unique_ptr<PacketSource> source  ///< [in] the source to wrap and take ownership of
    );

    OUSTER_API_FUNCTION
    PacketIterator begin() const override;

    OUSTER_API_FUNCTION
    PacketIterator end() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<SensorInfo>>& sensor_info() const override;

    OUSTER_API_FUNCTION
    bool is_live() const override;

    /// Get the PacketSource wrapped by this PacketSource
    /// @return wrapped PacketSource
    OUSTER_API_FUNCTION
    std::shared_ptr<PacketSource> child() const;

   protected:
    OUSTER_API_FUNCTION void close() override;
};

class MultiFrameSetIteratorImpl;
/// MultiFrameSetSource, combines multiple frame set sources into a single one.
class OUSTER_API_CLASS MultiFrameSetSource : public FrameSetSource {
    friend class MultiFrameSetIteratorImpl;

   public:
    /// Construct a MultiFrameSetSource from a provided sources
    OUSTER_API_FUNCTION
    MultiFrameSetSource(
        std::vector<std::shared_ptr<FrameSetSource>> sources  ///< [in] the sources to combine
    );

    OUSTER_API_FUNCTION
    FrameSetIterator begin() const override;

    OUSTER_API_FUNCTION
    FrameSetIterator begin(int idx) const override;

    OUSTER_API_FUNCTION
    FrameSetIterator end() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<SensorInfo>>& sensor_info() const override;

    OUSTER_API_FUNCTION
    bool is_live() const override;

    OUSTER_API_FUNCTION
    bool is_indexed() const override;

    OUSTER_API_FUNCTION
    size_t size() const override;

    OUSTER_API_FUNCTION
    size_t size_hint() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>& individual_index()
        const override;

    OUSTER_API_FUNCTION
    const std::vector<std::pair<uint64_t, uint64_t>>& full_index() const override;

    OUSTER_API_FUNCTION
    const std::vector<size_t>& frames_num() const override;

    OUSTER_API_FUNCTION
    std::unique_ptr<FrameSetSource> move() override;

   protected:
    OUSTER_API_FUNCTION void close() override;

    std::vector<std::shared_ptr<FrameSetSource>> sources_;

    bool indexed_;

    // mapping of original sensor info to sensor index and output sensor info
    std::map<const SensorInfo*, std::pair<int, std::shared_ptr<SensorInfo>>> info_map_;

    std::vector<std::shared_ptr<SensorInfo>> sensor_info_;

    struct OUSTER_API_CLASS IndexSample {
        int source_index;
        uint64_t timestamp;
        int frame_index;
    };
    std::vector<IndexSample> source_index_;
    std::vector<std::vector<std::pair<uint64_t, uint64_t>>> individual_index_;
    std::vector<std::pair<uint64_t, uint64_t>> full_index_;
    std::vector<size_t> frames_num_;

    size_t size_;
    size_t size_hint_;
};
}  // namespace core
}  // namespace sdk
}  // namespace ouster

#include "ouster/core/deprecation.h"

namespace ouster {
namespace sdk {
namespace core {

OUSTER_DEPRECATED_TYPE(ScanSourceWrapper, FrameSetSourceWrapper,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(AnyScanSource, AnyFrameSetSource, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(MultiScanSource, MultiFrameSetSource, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)

}  // namespace core
}  // namespace sdk
}  // namespace ouster
