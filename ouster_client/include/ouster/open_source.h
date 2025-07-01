/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <map>

#include "ouster/error_handler.h"
#include "ouster/impl/open_source_impl.h"
#include "ouster/io_type.h"
#include "ouster/packet_source.h"
#include "ouster/scan_source.h"
#include "ouster/scan_source_utils.h"

namespace ouster {

/// All possible configuration options for sources available through open_source
struct OUSTER_API_CLASS ScanSourceOptions {
    /// file to load extrinsics from
    impl::Parameter<std::string> extrinsics_file;

    /// list of extrinsics matrices to apply to the sensor with the same index,
    /// overrides any extrinsics loaded from file
    impl::Parameter<std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>>
        extrinsics;

    /// list of fields to decode into LidarScans, if not set decodes all fields,
    /// if an empty array decodes no fields
    // todo should we maybe include raw fields and raw headers in this?
    impl::Parameter<nonstd::optional<std::vector<std::string>>> field_names;

    /// If true, accept packets/scans that don't match the init_id/sn of the
    /// metadata
    impl::Parameter<bool> soft_id_check = false;

    /// if true, ensure that this file is indexed, indexing in place if
    /// necessary
    impl::Parameter<bool> index = false;

    /// optional list of metadata files to load with some formats, if not
    /// provided files are attempted to be found automatically
    impl::Parameter<std::vector<std::string>> meta;

    /// if set, the lidar udp port to use with the sensor
    impl::Parameter<nonstd::optional<uint16_t>> lidar_port;

    /// if set, the imu udp port to use with the sensor
    impl::Parameter<nonstd::optional<uint16_t>> imu_port;

    /// If true, do not change any settings on the sensor or reinitialize it
    impl::Parameter<bool> do_not_reinitialize = false;

    /// If true, do not change the udp_dest configuration option on the sensor
    impl::Parameter<bool> no_auto_udp_dest = false;

    /// timeout for each sensor in seconds
    /// only applies to iterators, not get_scan
    impl::Parameter<float> timeout = 1.0;

    /// timeout in seconds for http endpoints while configuring the sensor
    impl::Parameter<float> config_timeout = 45.0;

    /// maximum number of scans to queue, useful for limiting buffer bloat
    impl::Parameter<unsigned int> queue_size = 2;

    /// Override sensor info. If provided used instead of talking to the sensor.
    impl::Parameter<std::vector<ouster::sensor::sensor_info>> sensor_info;

    /// If true, batch raw_headers into each scan
    impl::Parameter<bool> raw_headers = false;

    /// If true, batch raw_fields into each scan
    impl::Parameter<bool> raw_fields = false;

    /// Configuration to apply to the sensors
    impl::Parameter<std::vector<ouster::sensor::sensor_config>> sensor_config;

    /// An optional error handler
    impl::Parameter<ouster::core::error_handler_t> error_handler{
        ouster::core::default_error_handler};

    /// Check if any parameters are unused
    /// @throw std::runtime_error if any parameters are unused
    OUSTER_API_FUNCTION
    void check(const char* source_type  ///<[in] name of source type
    ) const;
};

/// Construct a PacketSource for the given file with the specified options
/// @return constructed source
/// @throw std::runtime_error if source type is not detected or not supported
/// @throw std::runtime_error if provided option is not supported by source type
OUSTER_API_FUNCTION
ouster::core::AnyScanSource open_source(
    const std::string& source,  ///< [in] source filename(s)
    const std::function<void(ScanSourceOptions&)>& options =
        {},               ///< [in] source options
    bool collate = true,  ///< [in] whether to collate the source or not
    int sensor_idx = -1   ///< [in] sensor index to access in the data source
);

/// @copydoc ouster::open_source
OUSTER_API_FUNCTION
ouster::core::AnyScanSource open_source(
    const std::vector<std::string>& source,
    const std::function<void(ScanSourceOptions&)>& options = {},
    bool collate = true, int sensor_idx = -1);

/// All possible configuration options for packet sources available through
/// open_packet_source
struct OUSTER_API_CLASS PacketSourceOptions {
    /// file to load extrinsics from
    impl::Parameter<std::string> extrinsics_file;

    /// list of extrinsics matrices to apply to the sensor with the same index,
    /// overrides any extrinsics loaded from file
    impl::Parameter<std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>>
        extrinsics;

    /// If true, accept packets/scans that don't match the init_id/sn of the
    /// metadata
    impl::Parameter<bool> soft_id_check = false;

    /// optional list of metadata files to load with some formats, if not
    /// provided files are attempted to be found automatically
    impl::Parameter<std::vector<std::string>> meta;

    /// if set, the lidar udp port to use with the sensor
    impl::Parameter<nonstd::optional<uint16_t>> lidar_port;

    /// if set, the imu udp port to use with the sensor
    impl::Parameter<nonstd::optional<uint16_t>> imu_port;

    /// timeout for http endpoints while configuring the sensor
    impl::Parameter<float> config_timeout = 45.0;

    /// timeout for receiving packets from the sensor
    impl::Parameter<float> timeout = 1.0;

    /// If > 0.0 buffer packets for this long internally
    /// Useful if you may not be retrieving packets from the source consistently
    impl::Parameter<float> buffer_time_sec = 0.0;

    /// if true, ensure that this file is indexed, indexing in place if
    /// necessary
    impl::Parameter<bool> index = false;

    /// Override sensor info. If provided used instead of talking to the sensor.
    impl::Parameter<std::vector<ouster::sensor::sensor_info>> sensor_info;

    /// If true, do not change any settings on the sensor or reinitialize it
    impl::Parameter<bool> do_not_reinitialize = false;

    /// If true, do not change the udp_dest configuration option on the sensor
    impl::Parameter<bool> no_auto_udp_dest = false;

    /// Configuration to apply to the sensors
    impl::Parameter<std::vector<ouster::sensor::sensor_config>> sensor_config;

    /// Construct given the options in a ScanSourceOptions
    OUSTER_API_FUNCTION
    explicit PacketSourceOptions(ScanSourceOptions& options  ///< [in] options
    );

    /// Default constructor
    OUSTER_API_FUNCTION
    PacketSourceOptions();

    /// Check if any parameters are unused
    /// @throw std::runtime_error if any parameters are unused
    OUSTER_API_FUNCTION
    void check(const char* source_type  ///<[in] name of source type
    ) const;
};

/// Construct a PacketSource for the given file with the specified options
/// @return constructed source
/// @throw std::runtime_error if source type is not detected or not supported
/// @throw std::runtime_error if provided option is not supported by source type
OUSTER_API_FUNCTION
ouster::core::AnyPacketSource open_packet_source(
    const std::string& source,  ///< [in] source file name(s)
    const std::function<void(PacketSourceOptions&)>& options = {}
    ///< [in] source options
);

/// @copydoc ouster::open_packet_source
OUSTER_API_FUNCTION
ouster::core::AnyPacketSource open_packet_source(
    const std::vector<std::string>& source,
    const std::function<void(PacketSourceOptions&)>& options = {});

/// Populate the extrinsics in the sensor infos
OUSTER_API_FUNCTION
void populate_extrinsics(
    std::string extrinsics_file,  ///< [in] optional extrinsics file name
    std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
        extrinsics,  ///< [in] optional extrinsics list
    std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
        sensor_infos  ///< [out] sensor_infos to fill with extrinsics
);

/// Resolve list of field types for the given sensor infos
/// @return resolved field types
/// @throw std::runtime_error if field name not in list of default fields
OUSTER_API_FUNCTION
std::vector<std::vector<FieldType>> resolve_field_types(
    const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
        sensor_infos,  ///< [in] sensor_infos
    bool raw_headers,  ///< [in] if true, add raw headers
    bool raw_fields,   ///< [in] if true, add raw fields
    const nonstd::optional<std::vector<std::string>>&
        field_names  ///< [in] names of fields to include from list of defaults,
                     ///< if empty all default fields are added
);
}  // namespace ouster
