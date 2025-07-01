/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include "ouster/packet_source.h"
#include "ouster/scan_source.h"

namespace ouster {
namespace core {

/// ScanSource that collates a given ScanSource
class OUSTER_API_CLASS Collator : public ScanSource {
    const ScanSource* source_;
    uint64_t dt_;

    std::unique_ptr<ScanSource> parent_;

   public:
    /// Collate the provided source with the provided cut interval
    OUSTER_API_FUNCTION
    Collator(std::unique_ptr<ScanSource> source,  ///< [in] source to collate
             uint64_t dt_ns = 210000000  ///< [in] interval in nanoseconds after
                                         ///< which to cut collated scans
    );

    /// @copydoc Collator::Collator()
    OUSTER_API_FUNCTION
    Collator(const ScanSource& source, uint64_t dt_ns = 210000000);

    /// @copydoc Collator::Collator()
    OUSTER_API_FUNCTION
    Collator(ScanSource&& source, uint64_t dt_ns = 210000000);

    OUSTER_API_FUNCTION
    ScanIterator begin() const override;

    OUSTER_API_FUNCTION
    ScanIterator begin(int sensor_index) const override;

    OUSTER_API_FUNCTION
    ScanSource* move() override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
    sensor_info() const override;

    OUSTER_API_FUNCTION
    bool is_live() const override;

    OUSTER_API_FUNCTION
    bool is_indexed() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::pair<uint64_t, uint64_t>>& full_index()
        const override;

    OUSTER_API_FUNCTION
    const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
    individual_index() const override;

    OUSTER_API_FUNCTION
    const std::vector<size_t>& scans_num() const override;

   protected:
    void close() override;
};

/// Build a scan source that collates the provided source
/// @return scan source collating the provided source
OUSTER_API_FUNCTION
Collator collate(const ScanSource& source);

/// @copydoc collate()
OUSTER_API_FUNCTION
Collator collate(ScanSource&& source);

/// ScanSource that iterates over a single stream in a ScanSource
class OUSTER_API_CLASS Singler : public ScanSource {
    const ScanSource* source_;
    size_t idx_;
    std::unique_ptr<ScanSource> parent_;
    std::vector<std::shared_ptr<ouster::sensor::sensor_info>> sensor_info_;
    std::vector<size_t> scans_num_;

   public:
    /// Single the provided source, getting the sensor with the provided index
    OUSTER_API_FUNCTION
    Singler(const ScanSource& source,  ///< [in] source to single
            size_t idx                 ///< [in] index of sensor to single out
    );

    /// @copydoc Singler::Singler()
    OUSTER_API_FUNCTION
    Singler(ScanSource&& source, size_t idx);

    /// @copydoc Singler::Singler()
    OUSTER_API_FUNCTION
    Singler(std::unique_ptr<ScanSource> source, size_t idx);

    OUSTER_API_FUNCTION
    Singler(Singler&& copy) = default;

    OUSTER_API_FUNCTION
    ScanIterator begin() const override;

    OUSTER_API_FUNCTION
    ScanIterator begin(int idx) const override;

    OUSTER_API_FUNCTION
    ScanIterator end() const override;

    OUSTER_API_FUNCTION
    bool is_live() const override;

    OUSTER_API_FUNCTION
    bool is_indexed() const override;

    OUSTER_API_FUNCTION
    const std::vector<size_t>& scans_num() const override;

    OUSTER_API_FUNCTION
    ScanSource* move() override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
    sensor_info() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::pair<uint64_t, uint64_t>>& full_index()
        const override;

    OUSTER_API_FUNCTION
    const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
    individual_index() const override;

   private:
    void close() override;

    void build_index();

    std::vector<std::vector<std::pair<uint64_t, uint64_t>>> individual_index_;
    std::vector<std::pair<uint64_t, uint64_t>> full_index_;
};

/// ScanSource that slices a given ScanSource
class OUSTER_API_CLASS Slicer : public ScanSource {
    const ScanSource* source_;
    std::unique_ptr<ScanSource> parent_;

    int start_;
    int end_;
    int step_;

   public:
    /// Slice the provided source with the provided bounds
    OUSTER_API_FUNCTION
    Slicer(const ScanSource& source,  ///< [in] source to slice
           int start,                 ///< [in] start index of slice
           int end,                   ///< [in] end index of slice
           int step                   ///< [in] step size of slice
    );

    /// @copydoc Slicer::Slicer()
    OUSTER_API_FUNCTION
    Slicer(ScanSource&& source, int start, int end, int step);

    OUSTER_API_FUNCTION
    Slicer(Slicer&& source) = default;

    OUSTER_API_FUNCTION
    ScanIterator begin() const override;

    OUSTER_API_FUNCTION
    ScanIterator begin(int idx) const override;

    OUSTER_API_FUNCTION
    ScanSource* move() override;

    OUSTER_API_FUNCTION
    bool is_live() const override;

    OUSTER_API_FUNCTION
    bool is_indexed() const override;

    OUSTER_API_FUNCTION
    const std::vector<size_t>& scans_num() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
    sensor_info() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::pair<uint64_t, uint64_t>>& full_index()
        const override;

    OUSTER_API_FUNCTION
    const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
    individual_index() const override;

   private:
    void close() override;

    void build_index();

    std::vector<std::vector<std::pair<uint64_t, uint64_t>>> individual_index_;
    std::vector<std::pair<uint64_t, uint64_t>> full_index_;
    std::vector<size_t> scans_num_;
};

/// Wraps a shared pointer to a ScanSource to allow easy use of it by value
class OUSTER_API_CLASS AnyScanSource : public ScanSource {
    std::shared_ptr<ScanSource> source_;

   public:
    /// Construct an AnyScanSource from a provided source, taking ownership of
    /// it
    OUSTER_API_FUNCTION
    AnyScanSource(std::unique_ptr<ScanSource>
                      source  ///< [in] the source to wrap and take ownership of
    );

    OUSTER_API_FUNCTION
    ScanIterator begin() const override;

    OUSTER_API_FUNCTION
    ScanIterator begin(int idx) const override;

    OUSTER_API_FUNCTION
    ScanIterator end() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
    sensor_info() const override;

    OUSTER_API_FUNCTION
    bool is_live() const override;

    OUSTER_API_FUNCTION
    bool is_indexed() const override;

    OUSTER_API_FUNCTION
    size_t size() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
    individual_index() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::pair<uint64_t, uint64_t>>& full_index()
        const override;

    OUSTER_API_FUNCTION
    const std::vector<size_t>& scans_num() const override;

    OUSTER_API_FUNCTION
    ScanSource* move() override;

    /// Get the ScanSource wrapped by this ScanSource
    /// @return wrapped ScanSource
    OUSTER_API_FUNCTION
    std::shared_ptr<ScanSource> child() const;

   protected:
    void close() override;
};

/// Wraps a shared pointer to a ScanSource to allow easy use of it by value
class OUSTER_API_CLASS AnyPacketSource : public PacketSource {
    std::shared_ptr<PacketSource> source_;

   public:
    /// Construct an AnyPacketSource from a provided source, taking ownership of
    /// it
    OUSTER_API_FUNCTION
    AnyPacketSource(
        std::unique_ptr<PacketSource>
            source  ///< [in] the source to wrap and take ownership of
    );

    OUSTER_API_FUNCTION
    PacketIterator begin() const override;

    OUSTER_API_FUNCTION
    PacketIterator end() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
    sensor_info() const override;

    OUSTER_API_FUNCTION
    bool is_live() const override;

    /// Get the PacketSource wrapped by this PacketSource
    /// @return wrapped PacketSource
    OUSTER_API_FUNCTION
    std::shared_ptr<PacketSource> child() const;

   protected:
    void close() override;
};
}  // namespace core
}  // namespace ouster
