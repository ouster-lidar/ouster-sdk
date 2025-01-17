/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>

#include "nonstd/optional.hpp"
#include "ouster/metadata.h"
#include "ouster/types.h"

using namespace ouster::sensor;

inline ouster::mat4d mkmat4d(const std::array<double, 16>& v) {
    return Eigen::Map<const ouster::mat4d>(v.data()).transpose();
}

inline std::string getenvs(const std::string& var) {
    char* res = std::getenv(var.c_str());
    return res ? std::string{res} : std::string{};
}

void sinfo_populator(sensor_info& info, const std::string& /*name*/,
                     uint64_t sn, const std::string& fw_rev,
                     const lidar_mode mode, const std::string& prod_line,
                     const data_format& format,
                     const std::vector<double>& beam_azimuth_angles,
                     const std::vector<double> beam_altitude_angles,
                     const double lidar_origin_to_beam_origin_mm,
                     const ouster::mat4d& beam_to_lidar_transform,
                     const ouster::mat4d& imu_to_sensor_transform,
                     const ouster::mat4d& lidar_to_sensor_transform,
                     const ouster::mat4d& extrinsic, const int init_id,
                     const nonstd::optional<int> udp_port_lidar,
                     const nonstd::optional<int> udp_port_imu,
                     const std::string& build_date,
                     const std::string& image_rev, const std::string& prod_pn,
                     const std::string& status, const calibration_status& cal,
                     const sensor_config& config) {
    info.sn = sn;
    info.fw_rev = fw_rev;
    info.prod_line = prod_line;
    info.format = format;
    info.beam_azimuth_angles = beam_azimuth_angles;
    info.beam_altitude_angles = beam_altitude_angles;
    info.lidar_origin_to_beam_origin_mm = lidar_origin_to_beam_origin_mm;
    info.beam_to_lidar_transform = beam_to_lidar_transform;
    info.imu_to_sensor_transform = imu_to_sensor_transform;
    info.lidar_to_sensor_transform = lidar_to_sensor_transform;
    info.extrinsic = extrinsic;
    info.init_id = init_id;
    info.build_date = build_date;
    info.image_rev = image_rev;
    info.prod_pn = prod_pn;
    info.status = status;
    info.cal = cal;
    info.config = config;
    info.config.lidar_mode = mode;
    info.config.udp_port_lidar = udp_port_lidar;
    info.config.udp_port_imu = udp_port_imu;
}

static sensor_info si_1_12_os1_991913000010_64;
static sensor_info si_1_12_os1_991937000062_16A0_legacy;
static sensor_info si_1_12_os1_991937000062_64_legacy;
static sensor_info si_1_13_os1_991913000010_64;
static sensor_info si_1_13_os1_991937000062_16A0_legacy;
static sensor_info si_1_13_os1_991937000062_32A02_legacy;
static sensor_info si_1_13_os1_991937000062_64_legacy;
static sensor_info si_1_14_6cccd_os_882002000138_128_legacy;
static sensor_info si_1_14_6cccd_os_882002000138_32U0_legacy;
static sensor_info si_1_14_beta_os1_991937000062_16A0_legacy;
static sensor_info si_1_14_beta_os1_991937000062_64_legacy;
static sensor_info si_ouster_studio_reduced_config_v1;
static sensor_info si_2_0_rc2_os_992011000121_32U0_legacy;
static sensor_info si_2_0_0_os1_992008000494_128_col_win_legacy;
static sensor_info si_2_0_0_os1_991913000010_64;
static sensor_info si_2_1_2_os1_991913000010_64_legacy;
static sensor_info si_2_1_2_os1_991913000010_64;
static sensor_info si_2_2_os_992119000444_128_legacy;
static sensor_info si_2_2_os_992119000444_128;
static sensor_info si_2_3_1_os_992146000760_128_legacy;
static sensor_info si_2_3_1_os_992146000760_128;
static sensor_info si_2_4_0_os_992146000760_128;
static sensor_info si_2_4_0_os_992146000760_128_legacy;
static sensor_info si_2_5_0_os_992146000760_128;
static sensor_info si_2_5_0_os_992146000760_128_legacy;
static sensor_info si_3_0_1_os_122246000293_128_legacy;
static sensor_info si_3_0_1_os_122246000293_128;

// for lookup by name
const std::map<std::string, sensor_info*> expected_sensor_infos{
    {"1_12_os1-991913000010-64", &si_1_12_os1_991913000010_64},
    {"1_12_os1-991937000062-16A0_legacy",
     &si_1_12_os1_991937000062_16A0_legacy},
    {"1_12_os1-991937000062-64_legacy", &si_1_12_os1_991937000062_64_legacy},
    {"1_13_os1-991913000010-64", &si_1_13_os1_991913000010_64},
    {"1_13_os1-991937000062-16A0_legacy",
     &si_1_13_os1_991937000062_16A0_legacy},
    {"1_13_os1-991937000062-32A02_legacy",
     &si_1_13_os1_991937000062_32A02_legacy},
    {"1_14_6cccd_os-882002000138-128_legacy",
     &si_1_14_6cccd_os_882002000138_128_legacy},
    {"1_14_6cccd_os-882002000138-32U0_legacy",
     &si_1_14_6cccd_os_882002000138_32U0_legacy},
    {"1_14_beta_os1-991937000062-16A0_legacy",
     &si_1_14_beta_os1_991937000062_16A0_legacy},
    {"1_14_beta_os1-991937000062-64_legacy",
     &si_1_14_beta_os1_991937000062_64_legacy},
    {"ouster-studio-reduced-config-v1", &si_ouster_studio_reduced_config_v1},
    {"2_0_rc2_os-992011000121-32U0_legacy",
     &si_2_0_rc2_os_992011000121_32U0_legacy},
    {"2_0_0_os1-992008000494-128_col_win_legacy",
     &si_2_0_0_os1_992008000494_128_col_win_legacy},
    {"2_0_0_os1-991913000010-64", &si_2_0_0_os1_991913000010_64},
    {"2_1_2_os1-991913000010-64_legacy", &si_2_1_2_os1_991913000010_64_legacy},
    {"2_1_2_os1-991913000010-64", &si_2_1_2_os1_991913000010_64},
    {"2_2_os-992119000444-128_legacy", &si_2_2_os_992119000444_128_legacy},
    {"2_2_os-992119000444-128", &si_2_2_os_992119000444_128},
    {"2_3_1_os-992146000760-128_legacy", &si_2_3_1_os_992146000760_128_legacy},
    {"2_3_1_os-992146000760-128", &si_2_3_1_os_992146000760_128},
    {"2_4_0_os-992146000760-128", &si_2_4_0_os_992146000760_128},
    {"2_4_0_os-992146000760-128_legacy", &si_2_4_0_os_992146000760_128_legacy},
    {"2_5_0_os-992146000760-128", &si_2_5_0_os_992146000760_128},
    {"2_5_0_os-992146000760-128_legacy", &si_2_5_0_os_992146000760_128_legacy},
    {"3_0_1_os-122246000293-128", &si_3_0_1_os_122246000293_128},
    {"3_0_1_os-122246000293-128_legacy", &si_3_0_1_os_122246000293_128_legacy},
};
const std::map<std::string, std::vector<std::string>> expected_issues{
    {"1_12_os1-991913000010-64",
     {"$.sensor_info.initialization_id",
      "$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.udp_dest",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.columns_per_frame",
      "$.lidar_data_format.column_window.*",
      "$.lidar_data_format.column_window.*",
      "$.lidar_data_format.columns_per_packet",
      "$.lidar_data_format.pixels_per_column",
      "$.lidar_data_format.pixels_per_column",
      "$.lidar_data_format.pixel_shift_by_row.*",
      "$.lidar_data_format.udp_profile_lidar",
      "$.lidar_data_format.udp_profile_imu",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"1_12_os1-991937000062-64_legacy",
     {"$.sensor_info.initialization_id",
      "$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_port_imu",
      "$.config_params.udp_port_lidar",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.columns_per_frame",
      "$.lidar_data_format.column_window.*",
      "$.lidar_data_format.column_window.*",
      "$.lidar_data_format.columns_per_packet",
      "$.lidar_data_format.pixels_per_column",
      "$.lidar_data_format.pixels_per_column",
      "$.lidar_data_format.pixel_shift_by_row.*",
      "$.lidar_data_format.udp_profile_lidar",
      "$.lidar_data_format.udp_profile_imu",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"1_13_os1-991913000010-64",
     {"$.sensor_info.initialization_id",
      "$.config_params.columns_per_packet",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.udp_dest",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.columns_per_frame",
      "$.lidar_data_format.column_window.*",
      "$.lidar_data_format.column_window.*",
      "$.lidar_data_format.columns_per_packet",
      "$.lidar_data_format.pixels_per_column",
      "$.lidar_data_format.pixels_per_column",
      "$.lidar_data_format.pixel_shift_by_row.*",
      "$.lidar_data_format.udp_profile_lidar",
      "$.lidar_data_format.udp_profile_imu",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"1_14_6cccd_os-882002000138-128_legacy",
     {"$.sensor_info.initialization_id",
      "$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_port_imu",
      "$.config_params.udp_port_lidar",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.column_window.*",
      "$.lidar_data_format.udp_profile_lidar",
      "$.lidar_data_format.udp_profile_imu",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"1_14_6cccd_os-882002000138-32U0_legacy",
     {"$.sensor_info.initialization_id",
      "$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_port_imu",
      "$.config_params.udp_port_lidar",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.column_window.*",
      "$.lidar_data_format.udp_profile_lidar",
      "$.lidar_data_format.udp_profile_imu",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"1_14_beta_os1-991937000062-16A0_legacy",
     {"$.sensor_info.initialization_id",
      "$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_port_imu",
      "$.config_params.udp_port_lidar",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.column_window.*",
      "$.lidar_data_format.udp_profile_lidar",
      "$.lidar_data_format.udp_profile_imu",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"1_14_beta_os1-991937000062-64_legacy",
     {"$.sensor_info.initialization_id",
      "$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_port_imu",
      "$.config_params.udp_port_lidar",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.column_window.*",
      "$.lidar_data_format.udp_profile_lidar",
      "$.lidar_data_format.udp_profile_imu",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"ouster-studio-reduced-config-v1",
     {"$.sensor_info.build_date",
      "$.sensor_info.build_rev",
      "$.sensor_info.image_rev",
      "$.sensor_info.initialization_id",
      "$.sensor_info.prod_line",
      "$.sensor_info.prod_pn",
      "$.sensor_info.prod_sn",
      "$.sensor_info.status",
      "$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_port_imu",
      "$.config_params.udp_port_lidar",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.columns_per_frame",
      "$.lidar_data_format.column_window.*",
      "$.lidar_data_format.column_window.*",
      "$.lidar_data_format.columns_per_packet",
      "$.lidar_data_format.pixels_per_column",
      "$.lidar_data_format.pixel_shift_by_row.*.length()",
      "$.lidar_data_format.udp_profile_lidar",
      "$.lidar_data_format.udp_profile_imu",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.imu_intrinsics.imu_to_sensor_transform.*",
      "$.imu_intrinsics.imu_to_sensor_transform.*",
      "$.lidar_intrinsics.lidar_to_sensor_transform.*",
      "$.lidar_intrinsics.lidar_to_sensor_transform.*",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.lidar_origin_to_beam_origin_mm",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"2_0_rc2_os-992011000121-32U0_legacy",
     {"$.sensor_info.initialization_id",
      "$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_port_imu",
      "$.config_params.udp_port_lidar",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.udp_profile_lidar",
      "$.lidar_data_format.udp_profile_imu",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"2_0_0_os1-992008000494-128_col_win_legacy",
     {"$.sensor_info.initialization_id",
      "$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_port_imu",
      "$.config_params.udp_port_lidar",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.udp_profile_lidar",
      "$.lidar_data_format.udp_profile_imu",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"2_0_0_os1-991913000010-64",
     {"$.sensor_info.initialization_id",
      "$.config_params.columns_per_packet",
      "$.config_params.signal_multiplier",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.udp_profile_lidar",
      "$.lidar_data_format.udp_profile_imu",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"2_1_2_os1-991913000010-64_legacy",
     {"$.sensor_info.initialization_id",
      "$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_port_imu",
      "$.config_params.udp_port_lidar",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.udp_profile_lidar",
      "$.lidar_data_format.udp_profile_imu",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"2_1_2_os1-991913000010-64",
     {"$.sensor_info.initialization_id",
      "$.config_params.columns_per_packet",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.udp_profile_lidar",
      "$.lidar_data_format.udp_profile_imu",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"2_2_os-992119000444-128_legacy",
     {"$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"2_2_os-992119000444-128",
     {"$.config_params.gyro_fsr", "$.config_params.accel_fsr",
      "$.config_params.return_order", "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.fps", "$.lidar_data_format.fps",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*", "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"2_3_1_os-992146000760-128_legacy",
     {"$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"2_3_1_os-992146000760-128",
     {"$.config_params.gyro_fsr", "$.config_params.accel_fsr",
      "$.config_params.return_order", "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.fps", "$.lidar_data_format.fps",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*", "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"2_4_0_os-992146000760-128_legacy",
     {"$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"2_4_0_os-992146000760-128",
     {"$.config_params.gyro_fsr", "$.config_params.accel_fsr",
      "$.config_params.return_order", "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.fps", "$.lidar_data_format.fps",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.beam_intrinsics.beam_to_lidar_transform.*",
      "$.'ouster-sdk'.extrinsic.*", "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"2_5_0_os-992146000760-128_legacy",
     {"$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"2_5_0_os-992146000760-128",
     {"$.config_params.gyro_fsr", "$.config_params.accel_fsr",
      "$.config_params.return_order", "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.fps", "$.lidar_data_format.fps",
      "$.'ouster-sdk'.extrinsic.*", "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"3_0_1_os-122246000293-128_legacy",
     {"$.config_params.azimuth_window.*",
      "$.config_params.columns_per_packet",
      "$.config_params.multipurpose_io_mode",
      "$.config_params.nmea_baud_rate",
      "$.config_params.nmea_ignore_valid_char",
      "$.config_params.nmea_in_polarity",
      "$.config_params.nmea_leap_seconds",
      "$.config_params.operating_mode",
      "$.config_params.auto_start_flag",
      "$.config_params.auto_start_flag",
      "$.config_params.operating_mode",
      "$.config_params.phase_lock_enable",
      "$.config_params.phase_lock_offset",
      "$.config_params.signal_multiplier",
      "$.config_params.sync_pulse_in_polarity",
      "$.config_params.sync_pulse_out_angle",
      "$.config_params.sync_pulse_out_frequency",
      "$.config_params.sync_pulse_out_polarity",
      "$.config_params.sync_pulse_out_pulse_width",
      "$.config_params.timestamp_mode",
      "$.config_params.udp_dest",
      "$.config_params.udp_ip",
      "$.config_params.udp_profile_imu",
      "$.config_params.udp_profile_lidar",
      "$.config_params.gyro_fsr",
      "$.config_params.accel_fsr",
      "$.config_params.return_order",
      "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.fps",
      "$.lidar_data_format.fps",
      "$.calibration_status.reflectivity.timestamp",
      "$.calibration_status.reflectivity.valid",
      "$.'ouster-sdk'.extrinsic.*",
      "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}},
    {"3_0_1_os-122246000293-128",
     {"$.config_params.gyro_fsr", "$.config_params.accel_fsr",
      "$.config_params.return_order", "$.config_params.min_range_threshold_cm",
      "$.lidar_data_format.fps", "$.lidar_data_format.fps",
      "$.'ouster-sdk'.extrinsic.*", "$.'ouster-sdk'.extrinsic.*",
      "$.user_data"}}

};
// Add test data here: these are the base names for input .json files
// and populated structeds from populated_expected
//
// Expected output files will need to be re-generated whenever
// sensor_info or the parsing output changes
// clang-format off

//clang-format off
class MetaJsonTest : public testing::TestWithParam<const char*> {

    protected:
    static void SetUpTestSuite() {

        sensor_config config{};
        config.udp_dest="169.254.91.92";
        config.multipurpose_io_mode = MultipurposeIOMode::MULTIPURPOSE_OFF;
        config.nmea_baud_rate = NMEABaudRate::BAUD_9600;
        config.lidar_mode = lidar_mode::MODE_1024x10;
        config.timestamp_mode = timestamp_mode::TIME_FROM_INTERNAL_OSC;
        config.udp_port_imu = 60023;
        config.udp_port_lidar = 60823;
        config.sync_pulse_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.nmea_leap_seconds = 0;
        config.nmea_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.nmea_ignore_valid_char = false;
        config.sync_pulse_out_angle = 360;
        config.sync_pulse_out_frequency = 1;
        config.operating_mode = OperatingMode::OPERATING_NORMAL;
        config.sync_pulse_out_pulse_width = 10;
        sinfo_populator(si_1_12_os1_991913000010_64,
            "",
            991913000010,
            "v1.12.0",
            MODE_1024x10,
            "OS-1-64",
            {64, 16, 1024, {18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, }, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {3.073, 0.904, -1.261, -3.384, 3.048, 0.9, -1.235, -3.335, 3.029, 0.915, -1.2, -3.301, 3.026, 0.921, -1.179, -3.268, 3.032, 0.933, -1.156, -3.23, 3.028, 0.953, -1.131, -3.209, 3.059, 0.976, -1.112, -3.182, 3.071, 0.99, -1.087, -3.155, 3.097, 1.014, -1.103, -3.133, 3.118, 1.035, -1.039, -3.116, 3.144, 1.061, -1.013, -3.101, 3.178, 1.092, -0.993, -3.089, 3.217, 1.125, -0.972, -3.072, 3.265, 1.159, -0.949, -3.066, 3.321, 1.204, -0.921, -3.055, 3.381, 1.252, -0.898, -3.071},
            {16.729, 16.118, 15.543, 14.995, 14.526, 13.937, 13.381, 12.837, 12.378, 11.801, 11.251, 10.704, 10.251, 9.693, 9.138, 8.603, 8.149, 7.594, 7.049, 6.513, 6.056, 5.504, 4.961, 4.419, 3.967, 3.424, 2.881, 2.328, 1.88, 1.337, 0.787, 0.239, -0.205, -0.756, -1.39, -1.854, -2.3, -2.839, -3.386, -3.935, -4.391, -4.929, -5.472, -6.028, -6.471, -7.02, -7.563, -8.116, -8.572, -9.112, -9.66, -10.227, -10.687, -11.226, -11.777, -12.35, -12.807, -13.347, -13.902, -14.49, -14.968, -15.504, -16.078, -16.679},
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1, }),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1, }),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            60823,
            60023,
            "2019-05-02T20:17:32Z",
            "ousteros-image-prod-aries-v1.12.0-20190502211317",
            "840-101396-03",
            "RUNNING",
            calibration_status{},
            config
            );

        sinfo_populator(si_1_12_os1_991937000062_16A0_legacy,
            "os1-991937000062.local",
            991937000062,
            "v1.12.0",
            MODE_1024x10,
            "OS-1-16-A0",
            {16, 16, 1024, {18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, }, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {3.041, 0.895, -1.255, -3.387, 3.024, 0.897, -1.223, -3.326, 3.032, 0.916, -1.189, -3.288, 3.014, 0.926, -1.175, -3.243, 3.024, 0.956, -1.138, -3.206, 3.04, 0.958, -1.13, -3.183, 3.06, 0.991, -1.095, -3.159, 3.079, 1.003, -1.064, -3.129, 3.105, 1.037, -1.09, -3.1, 3.155, 1.06, -1.007, -3.083, 3.166, 1.087, -1.014, -3.067, 3.215, 1.11, -0.96, -3.048, 3.228, 1.145, -0.936, -3.04, 3.29, 1.188, -0.917, -3.037, 3.331, 1.222, -0.895, -3.043, 3.4, 1.265, -0.884, -3.041, },
            {16.509, 15.91, 15.345, 14.799, 14.329, 13.747, 13.185, 12.648, 12.192, 11.624, 11.081, 10.537, 10.083, 9.525, 8.97, 8.442, 7.984, 7.429, 6.892, 6.357, 5.899, 5.349, 4.811, 4.262, 3.828, 3.257, 2.726, 2.183, 1.733, 1.186, 0.64, 0.1, -0.349, -0.893, -1.499, -1.991, -2.444, -2.983, -3.532, -4.079, -4.53, -5.065, -5.642, -6.168, -6.616, -7.179, -7.708, -8.275, -8.707, -9.248, -9.8, -10.373, -10.836, -11.368, -11.92, -12.489, -12.966, -13.516, -14.077, -14.658, -15.121, -15.684, -16.238, -16.858, },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1, }),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1, }),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            {},
            {},
            "2019-05-02T20:17:32Z",
            "ousteros-image-prod-aries-v1.12.0-20190502211317",
            "840-101855-02",
            "RUNNING",
            calibration_status{},
            sensor_config{}
        );

        sinfo_populator(si_1_12_os1_991937000062_64_legacy,
            "os1-991937000062.local",
            991937000062,
            "v1.12.0",
            MODE_1024x10,
            "OS-1-64",
            {64, 16, 1024, {18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, }, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {3.041, 0.895, -1.255, -3.387, 3.024, 0.897, -1.223, -3.326, 3.032, 0.916, -1.189, -3.288, 3.014, 0.926, -1.175, -3.243, 3.024, 0.956, -1.138, -3.206, 3.04, 0.958, -1.13, -3.183, 3.06, 0.991, -1.095, -3.159, 3.079, 1.003, -1.064, -3.129, 3.105, 1.037, -1.09, -3.1, 3.155, 1.06, -1.007, -3.083, 3.166, 1.087, -1.014, -3.067, 3.215, 1.11, -0.96, -3.048, 3.228, 1.145, -0.936, -3.04, 3.29, 1.188, -0.917, -3.037, 3.331, 1.222, -0.895, -3.043, 3.4, 1.265, -0.884, -3.041, },
            {16.509, 15.91, 15.345, 14.799, 14.329, 13.747, 13.185, 12.648, 12.192, 11.624, 11.081, 10.537, 10.083, 9.525, 8.97, 8.442, 7.984, 7.429, 6.892, 6.357, 5.899, 5.349, 4.811, 4.262, 3.828, 3.257, 2.726, 2.183, 1.733, 1.186, 0.64, 0.1, -0.349, -0.893, -1.499, -1.991, -2.444, -2.983, -3.532, -4.079, -4.53, -5.065, -5.642, -6.168, -6.616, -7.179, -7.708, -8.275, -8.707, -9.248, -9.8, -10.373, -10.836, -11.368, -11.92, -12.489, -12.966, -13.516, -14.077, -14.658, -15.121, -15.684, -16.238, -16.858, },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1, }),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1, }),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            {},
            {},
            "2019-05-02T20:17:32Z",
            "ousteros-image-prod-aries-v1.12.0-20190502211317",
            "840-101855-02",
            "RUNNING",
            calibration_status{},
            sensor_config{}
        );

        config = sensor_config{};
        config.azimuth_window = std::make_pair(0, 36000);
        config.lidar_mode = lidar_mode::MODE_1024x10;
        config.multipurpose_io_mode = MultipurposeIOMode::MULTIPURPOSE_OFF;
        config.nmea_baud_rate = NMEABaudRate::BAUD_9600;
        config.nmea_ignore_valid_char = false;
        config.nmea_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.nmea_leap_seconds = 0;
        config.sync_pulse_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_angle = 360;
        config.sync_pulse_out_frequency = 1;
        config.sync_pulse_out_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_pulse_width = 10;
        config.timestamp_mode = timestamp_mode::TIME_FROM_INTERNAL_OSC;
        config.udp_dest="169.254.91.92";
        config.udp_port_imu = 7503;
        config.udp_port_lidar = 7502;
        config.operating_mode = OperatingMode::OPERATING_NORMAL;
        sinfo_populator(si_1_13_os1_991913000010_64,
            "",
            991913000010,
            "v1.13.0",
            MODE_1024x10,
            "OS-1-64",
            {64, 16, 1024, {18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, }, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {3.073, 0.904, -1.261, -3.384, 3.048, 0.9, -1.235, -3.335, 3.029, 0.915, -1.2, -3.301, 3.026, 0.921, -1.179, -3.268, 3.032, 0.933, -1.156, -3.23, 3.028, 0.953, -1.131, -3.209, 3.059, 0.976, -1.112, -3.182, 3.071, 0.99, -1.087, -3.155, 3.097, 1.014, -1.103, -3.133, 3.118, 1.035, -1.039, -3.116, 3.144, 1.061, -1.013, -3.101, 3.178, 1.092, -0.993, -3.089, 3.217, 1.125, -0.972, -3.072, 3.265, 1.159, -0.949, -3.066, 3.321, 1.204, -0.921, -3.055, 3.381, 1.252, -0.898, -3.071},
            {16.729, 16.118, 15.543, 14.995, 14.526, 13.937, 13.381, 12.837, 12.378, 11.801, 11.251, 10.704, 10.251, 9.693, 9.138, 8.603, 8.149, 7.594, 7.049, 6.513, 6.056, 5.504, 4.961, 4.419, 3.967, 3.424, 2.881, 2.328, 1.88, 1.337, 0.787, 0.239, -0.205, -0.756, -1.39, -1.854, -2.3, -2.839, -3.386, -3.935, -4.391, -4.929, -5.472, -6.028, -6.471, -7.02, -7.563, -8.116, -8.572, -9.112, -9.66, -10.227, -10.687, -11.226, -11.777, -12.35, -12.807, -13.347, -13.902, -14.49, -14.968, -15.504, -16.078, -16.679},
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1, }),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1, }),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            7502,
            7503,
            "2019-11-04T23:58:36Z",
            "ousteros-image-prod-aries-v1.13.0-20191105025459",
            "840-101396-03",
            "RUNNING",
            calibration_status{},
            config
        );

        sinfo_populator(si_1_13_os1_991937000062_32A02_legacy,
            "os1-991937000062.local",
            991937000062,
            "v1.13.0",
            MODE_1024x10,
            "OS-1-32-A02",
            {32, 16, 1024, {18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, }, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {0, 0.895, 0, -3.387, 0, 0.897, 0, -3.326, 0, 0.916, 0, -3.288, 0, 0.926, 0, -3.243, 0, 0.956, 0, -3.206, 0, 0.958, 0, -3.183, 0, 0.991, 0, -3.159, 0, 1.003, 0, -3.129, 0, 1.037, 0, -3.1, 0, 1.06, 0, -3.083, 0, 1.087, 0, -3.067, 0, 1.11, 0, -3.048, 0, 1.145, 0, -3.04, 0, 1.188, 0, -3.037, 0, 1.222, 0, -3.043, 0, 1.265, 0, -3.041, },
            {0, 15.91, 0, 14.799, 0, 13.747, 0, 12.648, 0, 11.624, 0, 10.537, 0, 9.525, 0, 8.442, 0, 7.429, 0, 6.357, 0, 5.349, 0, 4.262, 0, 3.257, 0, 2.183, 0, 1.186, 0, 0.1, 0, -0.893, 0, -1.991, 0, -2.983, 0, -4.079, 0, -5.065, 0, -6.168, 0, -7.179, 0, -8.275, 0, -9.248, 0, -10.373, 0, -11.368, 0, -12.489, 0, -13.516, 0, -14.658, 0, -15.684, 0, -16.858, },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1, }),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1, }),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            {},
            {},
            "2019-11-04T23:58:36Z",
            "ousteros-image-prod-aries-v1.13.0-20191105025459",
            "840-101855-02",
            "INITIALIZING",
            calibration_status{},
            {}
        );

        sinfo_populator(si_1_13_os1_991937000062_16A0_legacy,
            "os1-991937000062.local",
            991937000062,
            "v1.13.0",
            MODE_1024x10,
            "OS-1-16-A0",
            {16, 16, 1024, {18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, }, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {0, 0, 0, -3.387, 0, 0, 0, -3.326, 0, 0, 0, -3.288, 0, 0, 0, -3.243, 0, 0, 0, -3.206, 0, 0, 0, -3.183, 0, 0, 0, -3.159, 0, 0, 0, -3.129, 0, 0, 0, -3.1, 0, 0, 0, -3.083, 0, 0, 0, -3.067, 0, 0, 0, -3.048, 0, 0, 0, -3.04, 0, 0, 0, -3.037, 0, 0, 0, -3.043, 0, 0, 0, -3.041, },
            {0, 0, 0, 14.799, 0, 0, 0, 12.648, 0, 0, 0, 10.537, 0, 0, 0, 8.442, 0, 0, 0, 6.357, 0, 0, 0, 4.262, 0, 0, 0, 2.183, 0, 0, 0, 0.1, 0, 0, 0, -1.991, 0, 0, 0, -4.079, 0, 0, 0, -6.168, 0, 0, 0, -8.275, 0, 0, 0, -10.373, 0, 0, 0, -12.489, 0, 0, 0, -14.658, 0, 0, 0, -16.858, },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1, }),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1, }),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            {},
            {},
            "2019-11-04T23:58:36Z",
            "ousteros-image-prod-aries-v1.13.0-20191105025459",
            "840-101855-02",
            "RUNNING",
            calibration_status{},
            sensor_config{}
        );

        sinfo_populator(si_1_14_6cccd_os_882002000138_128_legacy,
            "os-882002000138.local",
            882002000138,
            "v1.14.0-beta.1-928-g6cccd78",
            MODE_1024x10,
            "OS-0-128",
            {128, 16, 1024, {48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, }, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {11.15, 3.677, -3.639, -10.81, 10.7, 3.534, -3.502, -10.41, 10.32, 3.412, -3.384, -10.08, 9.999, 3.307, -3.283, -9.786, 9.72, 3.217, -3.197, -9.537, 9.48, 3.14, -3.122, -9.322, 9.273, 3.074, -3.059, -9.138, 9.096, 3.017, -3.004, -8.981, 8.945, 2.968, -2.957, -8.847, 8.818, 2.927, -2.918, -8.736, 8.711, 2.894, -2.886, -8.643, 8.623, 2.866, -2.86, -8.57, 8.554, 2.844, -2.84, -8.513, 8.501, 2.828, -2.825, -8.472, 8.465, 2.817, -2.815, -8.447, 8.444, 2.811, -2.811, -8.438, 8.438, 2.811, -2.811, -8.444, 8.447, 2.815, -2.817, -8.465, 8.472, 2.825, -2.828, -8.501, 8.513, 2.84, -2.844, -8.554, 8.57, 2.86, -2.866, -8.623, 8.643, 2.886, -2.894, -8.711, 8.736, 2.918, -2.927, -8.818, 8.847, 2.957, -2.968, -8.945, 8.981, 3.004, -3.017, -9.096, 9.138, 3.059, -3.074, -9.273, 9.322, 3.122, -3.14, -9.48, 9.537, 3.197, -3.217, -9.72, 9.786, 3.283, -3.307, -9.999, 10.08, 3.384, -3.412, -10.32, 10.41, 3.502, -3.534, -10.7, 10.81, 3.639, -3.677, -11.15, },
            {46.2, 45.12, 44.36, 43.9, 43.14, 42.09, 41.33, 40.87, 40.11, 39.09, 38.34, 37.86, 37.11, 36.11, 35.37, 34.88, 34.14, 33.16, 32.43, 31.92, 31.19, 30.24, 29.51, 28.99, 28.26, 27.33, 26.61, 26.07, 25.34, 24.44, 23.72, 23.17, 22.45, 21.57, 20.86, 20.29, 19.57, 18.72, 18, 17.42, 16.7, 15.87, 15.16, 14.56, 13.84, 13.04, 12.33, 11.71, 10.99, 10.21, 9.503, 8.862, 8.152, 7.388, 6.684, 6.023, 5.314, 4.572, 3.868, 3.188, 2.479, 1.758, 1.055, 0.3541, -0.3541, -1.055, -1.758, -2.479, -3.188, -3.868, -4.572, -5.314, -6.023, -6.684, -7.388, -8.152, -8.862, -9.503, -10.21, -10.99, -11.71, -12.33, -13.04, -13.84, -14.56, -15.16, -15.87, -16.7, -17.42, -18, -18.72, -19.57, -20.29, -20.86, -21.57, -22.45, -23.17, -23.72, -24.44, -25.34, -26.07, -26.61, -27.33, -28.26, -28.99, -29.51, -30.24, -31.19, -31.92, -32.43, -33.16, -34.14, -34.88, -35.37, -36.11, -37.11, -37.86, -38.34, -39.09, -40.11, -40.87, -41.33, -42.09, -43.14, -43.9, -44.36, -45.12, -46.2, },
            27.67,
            mkmat4d({1, 0, 0, 27.67, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1, }),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1, }),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            {},
            {},
            "2020-02-13T17:22:02Z",
            "ousteros-image-prod-aries-v1.14.0-beta.1+ci+git+beta2@6cccd783ea+dev_fw_PR-884-20200213171907-staging",
            "840-102144-A",
            "RUNNING",
            calibration_status{},
            sensor_config{}
        );

        sinfo_populator(si_1_14_6cccd_os_882002000138_32U0_legacy,
            "os-882002000138.local",
            882002000138,
            "v1.14.0-beta.1-928-g6cccd78",
            MODE_1024x10,
            "OS-0-32-U0",
            {32, 16, 1024, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, }, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {-10.81, -10.41, -10.08, -9.786, -9.537, -9.322, -9.138, -8.981, -8.847, -8.736, -8.643, -8.57, -8.513, -8.472, -8.447, -8.438, -8.444, -8.465, -8.501, -8.554, -8.623, -8.711, -8.818, -8.945, -9.096, -9.273, -9.48, -9.72, -9.999, -10.32, -10.7, -11.15, },
            {43.9, 40.87, 37.86, 34.88, 31.92, 28.99, 26.07, 23.17, 20.29, 17.42, 14.56, 11.71, 8.862, 6.023, 3.188, 0.3541, -2.479, -5.314, -8.152, -10.99, -13.84, -16.7, -19.57, -22.45, -25.34, -28.26, -31.19, -34.14, -37.11, -40.11, -43.14, -46.2, },
            27.67,
            mkmat4d({1, 0, 0, 27.67, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1, }),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1, }),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            {},
            {},
            "2020-02-13T17:22:02Z",
            "ousteros-image-prod-aries-v1.14.0-beta.1+ci+git+beta2@6cccd783ea+dev_fw_PR-884-20200213171907-staging",
            "840-102144-A",
            "RUNNING",
            calibration_status{},
            sensor_config{}
        );

        sinfo_populator( si_1_14_beta_os1_991937000062_16A0_legacy,
            "os1-991937000062.local",
            991937000062,
            "v1.14.0-beta.1-87-gde6f92c",
            MODE_1024x10,
            "OS-1-16-A0",
            {16, 16, 1024, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, }, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {-3.387, -3.326, -3.288, -3.243, -3.206, -3.183, -3.159, -3.129, -3.1, -3.083, -3.067, -3.048, -3.04, -3.037, -3.043, -3.041, },
            {14.799, 12.648, 10.537, 8.442, 6.357, 4.262, 2.183, 0.1, -1.991, -4.079, -6.168, -8.275, -10.373, -12.489, -14.658, -16.858, },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1, }),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1, }),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            {},
            {},
            "2020-02-11T00:14:48Z",
            "ousteros-image-prod-aries-v1.14.0-beta.1+ci+git+master@de6f92cb16-20200211001242-staging",
            "840-101855-02",
            "RUNNING",
            calibration_status{},
            sensor_config{}
        );

        sinfo_populator(si_1_14_beta_os1_991937000062_64_legacy,
            "os1-991937000062.local",
            991937000062,
            "v1.14.0-beta.1-87-gde6f92c",
            MODE_1024x10,
            "OS-1-64",
            {64, 16, 1024, {18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, }, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {3.041, 0.895, -1.255, -3.387, 3.024, 0.897, -1.223, -3.326, 3.032, 0.916, -1.189, -3.288, 3.014, 0.926, -1.175, -3.243, 3.024, 0.956, -1.138, -3.206, 3.04, 0.958, -1.13, -3.183, 3.06, 0.991, -1.095, -3.159, 3.079, 1.003, -1.064, -3.129, 3.105, 1.037, -1.09, -3.1, 3.155, 1.06, -1.007, -3.083, 3.166, 1.087, -1.014, -3.067, 3.215, 1.11, -0.96, -3.048, 3.228, 1.145, -0.936, -3.04, 3.29, 1.188, -0.917, -3.037, 3.331, 1.222, -0.895, -3.043, 3.4, 1.265, -0.884, -3.041, },
            {16.509, 15.91, 15.345, 14.799, 14.329, 13.747, 13.185, 12.648, 12.192, 11.624, 11.081, 10.537, 10.083, 9.525, 8.97, 8.442, 7.984, 7.429, 6.892, 6.357, 5.899, 5.349, 4.811, 4.262, 3.828, 3.257, 2.726, 2.183, 1.733, 1.186, 0.64, 0.1, -0.349, -0.893, -1.499, -1.991, -2.444, -2.983, -3.532, -4.079, -4.53, -5.065, -5.642, -6.168, -6.616, -7.179, -7.708, -8.275, -8.707, -9.248, -9.8, -10.373, -10.836, -11.368, -11.92, -12.489, -12.966, -13.516, -14.077, -14.658, -15.121, -15.684, -16.238, -16.858, },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1, }),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1, }),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            {},
            {},
            "2020-02-11T00:14:48Z",
            "ousteros-image-prod-aries-v1.14.0-beta.1+ci+git+master@de6f92cb16-20200211001242-staging",
            "840-101855-02",
            "INITIALIZING",
            calibration_status{},
            sensor_config{}
        );

        sinfo_populator(si_ouster_studio_reduced_config_v1,
            "",
            0,
            "",
            MODE_2048x10,
            "",
            {64, 16, 2048, {36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, 36, 24, 12, 0, },  {0, 2047}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {1.5, -4.16, 1.49, -4.15, 1.48, -4.15, 1.47, -4.15, 1.46, -4.16, 1.47, -4.18, 1.45, -4.16, 1.45, -4.18, 1.45, -4.19, 1.43, -4.18, 1.43, -4.2, 1.42, -4.2, 1.41, -4.21, 1.41, -4.22, 1.39, -4.21, 1.4, -4.22, 1.39, -4.25, 1.39, -4.24, 1.39, -4.25, 1.38, -4.26, 1.38, -4.27, 1.37, -4.25, 1.37, -4.27, 1.37, -4.28, 1.36, -4.27, 1.37, -4.27, 1.35, -4.29, 1.35, -4.28, 1.34, -4.3, 1.33, -4.3, 1.32, -4.31, 1.34, -4.31, },
            {21.45, 20.84, 20.21, 19.6, 18.95, 18.34, 17.69, 17.06, 16.38, 15.76, 15.09, 14.43, 13.74, 13.11, 12.41, 11.77, 11.06, 10.4, 9.68, 9.03, 8.31, 7.64, 6.92, 6.26, 5.53, 4.86, 4.14, 3.46, 2.72, 2.06, 1.33, 0.67, -0.08, -0.77, -1.49, -2.16, -2.88, -3.58, -4.29, -4.96, -5.68, -6.36, -7.07, -7.74, -8.45, -9.13, -9.81, -10.5, -11.19, -11.84, -12.53, -13.19, -13.88, -14.53, -15.21, -15.83, -16.52, -17.14, -17.82, -18.43, -19.1, -19.68, -20.33, -20.92, },
            12.163,
            mkmat4d({1, 0, 0, 12.163, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1, }),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1, }),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            {},
            {},
            "",
            "",
            "",
            "",
            calibration_status{},
            sensor_config{}
        );

        sinfo_populator(si_2_0_rc2_os_992011000121_32U0_legacy,
            "os-992011000121.local",
            992011000121,
            "v2.0.0-rc.2",
            MODE_512x10,
            "OS-1-32-U0",
            {32, 16, 512, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, }, {0, 511}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {-4.252, -4.2518, -4.2517, -4.2518, -4.2521, -4.2526, -4.2534, -4.2543, -4.2555, -4.2568, -4.2582, -4.2596, -4.2608, -4.2618, -4.2625, -4.2628, -4.2626, -4.262, -4.2611, -4.2599, -4.2586, -4.2572, -4.2558, -4.2546, -4.2536, -4.2528, -4.2522, -4.2518, -4.2517, -4.2517, -4.2519, -4.2521, },
            {20.5477, 19.2919, 18.0171, 16.7242, 15.4141, 14.088, 12.7466, 11.3909, 10.022, 8.641, 7.249, 5.8474, 4.4377, 3.0216, 1.601, 0.1779, -1.2454, -2.6668, -4.0842, -5.4956, -6.8994, -8.2939, -9.6778, -11.0499, -12.409, -13.754, -15.0841, -16.3982, -17.6955, -18.975, -20.2356, -21.4764, },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1, }),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1, }),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            {},
            {},
            "2020-10-23T14:05:18Z",
            "ousteros-image-prod-aries-v2.0.0-rc.2+20201023140416.staging",
            "840-102145-B",
            "RUNNING",
            calibration_status{},
            sensor_config{}
        );

        sinfo_populator(si_2_0_0_os1_992008000494_128_col_win_legacy,
            "192.168.87.249",
            992008000494,
            "v2.0.0",
            MODE_2048x10,
            "OS-1-128",
            {128, 16, 2048, {48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, 48, 32, 16, 0, }, {2014, 2036}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {4.2521, 1.4197, -1.4196, -4.252, 4.2519, 1.4196, -1.4196, -4.2518, 4.2517, 1.4196, -1.4195, -4.2517, 4.2517, 1.4196, -1.4196, -4.2518, 4.2518, 1.4197, -1.4197, -4.2521, 4.2522, 1.4198, -1.4199, -4.2526, 4.2528, 1.4201, -1.4202, -4.2534, 4.2536, 1.4204, -1.4205, -4.2543, 4.2546, 1.4208, -1.421, -4.2555, 4.2558, 1.4213, -1.4215, -4.2568, 4.2572, 1.4219, -1.422, -4.2582, 4.2586, 1.4224, -1.4225, -4.2596, 4.2599, 1.4229, -1.423, -4.2608, 4.2611, 1.4234, -1.4234, -4.2618, 4.262, 1.4237, -1.4237, -4.2625, 4.2626, 1.4239, -1.4239, -4.2628, 4.2628, 1.4239, -1.4239, -4.2626, 4.2625, 1.4237, -1.4237, -4.262, 4.2618, 1.4234, -1.4234, -4.2611, 4.2608, 1.423, -1.4229, -4.2599, 4.2596, 1.4225, -1.4224, -4.2586, 4.2582, 1.422, -1.4219, -4.2572, 4.2568, 1.4215, -1.4213, -4.2558, 4.2555, 1.421, -1.4208, -4.2546, 4.2543, 1.4205, -1.4204, -4.2536, 4.2534, 1.4202, -1.4201, -4.2528, 4.2526, 1.4199, -1.4198, -4.2522, 4.2521, 1.4197, -1.4197, -4.2518, 4.2518, 1.4196, -1.4196, -4.2517, 4.2517, 1.4195, -1.4196, -4.2517, 4.2518, 1.4196, -1.4196, -4.2519, 4.252, 1.4196, -1.4197, -4.2521, },
            {21.4764, 21.1679, 20.8583, 20.5477, 20.2356, 19.9221, 19.6075, 19.2919, 18.975, 18.6567, 18.3375, 18.0171, 17.6955, 17.3729, 17.0492, 16.7242, 16.3982, 16.0716, 15.7437, 15.4141, 15.0841, 14.7537, 14.4218, 14.088, 13.754, 13.4202, 13.0844, 12.7466, 12.409, 12.0718, 11.7326, 11.3909, 11.0499, 10.7097, 10.3671, 10.022, 9.6778, 9.3348, 8.9892, 8.641, 8.2939, 7.9482, 7.5999, 7.249, 6.8994, 6.5512, 6.2005, 5.8474, 5.4956, 5.1452, 4.7925, 4.4377, 4.0842, 3.7318, 3.3775, 3.0216, 2.6668, 2.3127, 1.9573, 1.601, 1.2454, 0.89, 0.5341, 0.1779, -0.1779, -0.5341, -0.89, -1.2454, -1.601, -1.9573, -2.3127, -2.6668, -3.0216, -3.3775, -3.7318, -4.0842, -4.4377, -4.7925, -5.1452, -5.4956, -5.8474, -6.2005, -6.5512, -6.8994, -7.249, -7.5999, -7.9482, -8.2939, -8.641, -8.9892, -9.3348, -9.6778, -10.022, -10.3671, -10.7097, -11.0499, -11.3909, -11.7326, -12.0718, -12.409, -12.7466, -13.0844, -13.4202, -13.754, -14.088, -14.4218, -14.7537, -15.0841, -15.4141, -15.7437, -16.0716, -16.3982, -16.7242, -17.0492, -17.3729, -17.6955, -18.0171, -18.3375, -18.6567, -18.975, -19.2919, -19.6075, -19.9221, -20.2356, -20.5477, -20.8583, -21.1679, -21.4764, },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1, }),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1, }),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            {},
            {},
            "2020-11-24T06:51:26Z",
            "ousteros-image-prod-aries-v2.0.0+20201124065024",
            "840-102145-A",
            "RUNNING",
            calibration_status{},
            sensor_config{}
        );

        config = sensor_config{};
        config.operating_mode = OperatingMode::OPERATING_NORMAL;
        config.azimuth_window = std::make_pair<int, int>(0, 360000);
        config.lidar_mode = lidar_mode::MODE_1024x10;
        config.multipurpose_io_mode = MultipurposeIOMode::MULTIPURPOSE_OFF;
        config.nmea_baud_rate = NMEABaudRate::BAUD_9600;
        config.nmea_ignore_valid_char = false;
        config.nmea_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.nmea_leap_seconds = 0;
        config.phase_lock_enable = false;
        config.phase_lock_offset = 0;
        config.sync_pulse_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_angle = 360;
        config.sync_pulse_out_frequency = 1;
        config.sync_pulse_out_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_pulse_width = 10;
        config.timestamp_mode = timestamp_mode::TIME_FROM_INTERNAL_OSC;
        config.udp_dest="169.254.91.92";
        config.udp_port_imu = 55824;
        config.udp_port_lidar = 55426;
        sinfo_populator( si_2_0_0_os1_991913000010_64,
            "",
            991913000010,
            "v2.0.0",
            MODE_1024x10,
            "OS-1-64",
            {64, 16, 1024, { 19, 13, 6, 0, 19, 13, 6, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 20, 14, 7, 1}, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            { 3.073, 0.904, -1.261, -3.384, 3.048, 0.9, -1.235, -3.335, 3.029, 0.915, -1.2, -3.301, 3.026, 0.921, -1.179, -3.268, 3.032, 0.933, -1.156, -3.23, 3.028, 0.953, -1.131, -3.209, 3.059, 0.976, -1.112, -3.182, 3.071, 0.99, -1.087, -3.155, 3.097, 1.014, -1.103, -3.133, 3.118, 1.035, -1.039, -3.116, 3.144, 1.061, -1.013, -3.101, 3.178, 1.092, -0.993, -3.089, 3.217, 1.125, -0.972, -3.072, 3.265, 1.159, -0.949, -3.066, 3.321, 1.204, -0.921, -3.055, 3.381, 1.252, -0.898, -3.071},
            { 16.729, 16.118, 15.543, 14.995, 14.526, 13.937, 13.381, 12.837, 12.378, 11.801, 11.251, 10.704, 10.251, 9.693, 9.138, 8.603, 8.149, 7.594, 7.049, 6.513, 6.056, 5.504, 4.961, 4.419, 3.967, 3.424, 2.881, 2.328, 1.88, 1.337, 0.787, 0.239, -0.205, -0.756, -1.39, -1.854, -2.3, -2.839, -3.386, -3.935, -4.391, -4.929, -5.472, -6.028, -6.471, -7.02, -7.563, -8.116, -8.572, -9.112, -9.66, -10.227, -10.687, -11.226, -11.777, -12.35, -12.807, -13.347, -13.902, -14.49, -14.968, -15.504, -16.078, -16.679},
            12.163,
            mkmat4d({1, 0, 0, 12.163, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({ 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1,}),
            mkmat4d({ -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            55426,
            55824,
            "2020-11-24T06:51:26Z",
            "ousteros-image-prod-aries-v2.0.0+20201124065024",
            "840-101396-03",
            "RUNNING",
            calibration_status(),
            config
        );

        sinfo_populator(si_2_1_2_os1_991913000010_64_legacy,
            "",
            991913000010,
            "v2.1.2",
            MODE_1024x10,
            "OS-1-64",
            {64, 16, 1024, { 19, 13, 6, 0, 19, 13, 6, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 20, 14, 7, 1}, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            { 3.073, 0.904, -1.261, -3.384, 3.048, 0.9, -1.235, -3.335, 3.029, 0.915, -1.2, -3.301, 3.026, 0.921, -1.179, -3.268, 3.032, 0.933, -1.156, -3.23, 3.028, 0.953, -1.131, -3.209, 3.059, 0.976, -1.112, -3.182, 3.071, 0.99, -1.087, -3.155, 3.097, 1.014, -1.103, -3.133, 3.118, 1.035, -1.039, -3.116, 3.144, 1.061, -1.013, -3.101, 3.178, 1.092, -0.993, -3.089, 3.217, 1.125, -0.972, -3.072, 3.265, 1.159, -0.949, -3.066, 3.321, 1.204, -0.921, -3.055, 3.381, 1.252, -0.898, -3.071},
            { 16.729, 16.118, 15.543, 14.995, 14.526, 13.937, 13.381, 12.837, 12.378, 11.801, 11.251, 10.704, 10.251, 9.693, 9.138, 8.603, 8.149, 7.594, 7.049, 6.513, 6.056, 5.504, 4.961, 4.419, 3.967, 3.424, 2.881, 2.328, 1.88, 1.337, 0.787, 0.239, -0.205, -0.756, -1.39, -1.854, -2.3, -2.839, -3.386, -3.935, -4.391, -4.929, -5.472, -6.028, -6.471, -7.02, -7.563, -8.116, -8.572, -9.112, -9.66, -10.227, -10.687, -11.226, -11.777, -12.35, -12.807, -13.347, -13.902, -14.49, -14.968, -15.504, -16.078, -16.679},
            12.163,
            mkmat4d({1, 0, 0, 12.163, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({ 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1,}),
            mkmat4d({ -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            {},
            {},
            "2021-07-27T22:34:53Z",
            "ousteros-image-prod-aries-v2.1.2+20210727223313.patch-v2.1.2",
            "840-101396-03",
            "RUNNING",
            calibration_status(),
            sensor_config{}
        );

        config = sensor_config{};
        config.operating_mode = OperatingMode::OPERATING_NORMAL;
        config.azimuth_window = std::make_pair<int, int>(0, 360000);
        config.lidar_mode = lidar_mode::MODE_1024x10;
        config.multipurpose_io_mode = MultipurposeIOMode::MULTIPURPOSE_OFF;
        config.nmea_baud_rate = NMEABaudRate::BAUD_9600;
        config.nmea_ignore_valid_char = false;
        config.nmea_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.nmea_leap_seconds = 0;
        config.phase_lock_enable = false;
        config.phase_lock_offset = 0;
        config.signal_multiplier = 1;
        config.sync_pulse_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_angle = 360;
        config.sync_pulse_out_frequency = 1;
        config.sync_pulse_out_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_pulse_width = 10;
        config.timestamp_mode = timestamp_mode::TIME_FROM_INTERNAL_OSC;
        config.udp_port_imu = 7503;
        config.udp_port_lidar = 7502; 
        sinfo_populator(si_2_1_2_os1_991913000010_64,
            "",
            991913000010,
            "v2.1.2",
            MODE_1024x10,
            "OS-1-64",
            {64, 16, 1024, { 19, 13, 6, 0, 19, 13, 6, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 19, 13, 7, 1, 20, 14, 7, 1}, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            { 3.073, 0.904, -1.261, -3.384, 3.048, 0.9, -1.235, -3.335, 3.029, 0.915, -1.2, -3.301, 3.026, 0.921, -1.179, -3.268, 3.032, 0.933, -1.156, -3.23, 3.028, 0.953, -1.131, -3.209, 3.059, 0.976, -1.112, -3.182, 3.071, 0.99, -1.087, -3.155, 3.097, 1.014, -1.103, -3.133, 3.118, 1.035, -1.039, -3.116, 3.144, 1.061, -1.013, -3.101, 3.178, 1.092, -0.993, -3.089, 3.217, 1.125, -0.972, -3.072, 3.265, 1.159, -0.949, -3.066, 3.321, 1.204, -0.921, -3.055, 3.381, 1.252, -0.898, -3.071},
            { 16.729, 16.118, 15.543, 14.995, 14.526, 13.937, 13.381, 12.837, 12.378, 11.801, 11.251, 10.704, 10.251, 9.693, 9.138, 8.603, 8.149, 7.594, 7.049, 6.513, 6.056, 5.504, 4.961, 4.419, 3.967, 3.424, 2.881, 2.328, 1.88, 1.337, 0.787, 0.239, -0.205, -0.756, -1.39, -1.854, -2.3, -2.839, -3.386, -3.935, -4.391, -4.929, -5.472, -6.028, -6.471, -7.02, -7.563, -8.116, -8.572, -9.112, -9.66, -10.227, -10.687, -11.226, -11.777, -12.35, -12.807, -13.347, -13.902, -14.49, -14.968, -15.504, -16.078, -16.679},
            12.163,
            mkmat4d({1, 0, 0, 12.163, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({ 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1,}),
            mkmat4d({ -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            0,
            7502,
            7503,
            "2021-07-27T22:34:53Z",
            "ousteros-image-prod-aries-v2.1.2+20210727223313.patch-v2.1.2",
            "840-101396-03",
            "RUNNING",
            calibration_status{false, nonstd::nullopt},
            config
        );

        sinfo_populator(si_2_2_os_992119000444_128_legacy,
            "",
            992119000444,
            "v2.2.0-rc.1",
            MODE_1024x10,
            "OS-1-128",
            {128, 16, 1024, {24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0}, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {4.26, 1.44, -1.39, -4.22, 4.27, 1.45, -1.38, -4.22, 4.26, 1.45, -1.38, -4.2, 4.25, 1.43, -1.4, -4.22, 4.28, 1.44, -1.4, -4.22, 4.27, 1.45, -1.4, -4.22, 4.27, 1.44, -1.39, -4.24, 4.26, 1.43, -1.39, -4.24, 4.26, 1.44, -1.4, -4.24, 4.25, 1.4, -1.4, -4.25, 4.25, 1.44, -1.41, -4.25, 4.28, 1.42, -1.4, -4.24, 4.26, 1.41, -1.43, -4.25, 4.26, 1.41, -1.42, -4.24, 4.24, 1.43, -1.42, -4.26, 4.27, 1.42, -1.42, -4.27, 4.26, 1.41, -1.45, -4.24, 4.24, 1.42, -1.41, -4.24, 4.25, 1.42, -1.43, -4.25, 4.27, 1.39, -1.43, -4.26, 4.23, 1.39, -1.44, -4.27, 4.26, 1.41, -1.43, -4.26, 4.24, 1.4, -1.42, -4.28, 4.24, 1.42, -1.43, -4.26, 4.24, 1.4, -1.43, -4.26, 4.22, 1.4, -1.43, -4.27, 4.24, 1.41, -1.44, -4.28, 4.24, 1.4, -1.45, -4.29, 4.23, 1.39, -1.45, -4.27, 4.22, 1.38, -1.45, -4.27, 4.2, 1.39, -1.45, -4.29, 4.22, 1.39, -1.45, -4.29},
            {21.51, 21.18, 20.87, 20.57, 20.26, 19.94, 19.63, 19.32, 19.01, 18.68, 18.35, 18.05, 17.72, 17.38, 17.06, 16.74, 16.42, 16.09, 15.76, 15.44, 15.09, 14.77, 14.43, 14.1, 13.75, 13.42, 13.1, 12.77, 12.41, 12.06, 11.73, 11.4, 11.04, 10.71, 10.36, 10.03, 9.67, 9.32, 9.01, 8.65, 8.28, 7.94, 7.59, 7.26, 6.91, 6.53, 6.21, 5.87, 5.5, 5.15, 4.8, 4.46, 4.11, 3.74, 3.39, 3.06, 2.68, 2.34, 1.99, 1.62, 1.27, 0.92, 0.58, 0.21, -0.14, -0.5, -0.86, -1.18, -1.55, -1.91, -2.25, -2.59, -2.98, -3.33, -3.68, -4.01, -4.38, -4.74, -5.08, -5.44, -5.8, -6.14, -6.48, -6.83, -7.19, -7.54, -7.88, -8.22, -8.59, -8.94, -9.26, -9.63, -9.97, -10.3, -10.65, -10.97, -11.33, -11.68, -12.02, -12.36, -12.72, -13.06, -13.37, -13.7, -14.05, -14.39, -14.71, -15.06, -15.39, -15.72, -16.04, -16.38, -16.7, -17.03, -17.34, -17.67, -18.01, -18.33, -18.64, -18.93, -19.29, -19.59, -19.89, -20.2, -20.51, -20.82, -21.14, -21.44},
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1}),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1}),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            7109745,
            7502,
            7503,
            "2021-09-30T15:58:18Z",
            "ousteros-image-prod-aries-v2.2.0-rc.1",
            "840-102145-05",
            "RUNNING",
            calibration_status{},
            sensor_config{}
        );

        config = sensor_config{};
        config.operating_mode = OperatingMode::OPERATING_NORMAL;
        config.azimuth_window = std::make_pair<int, int>(0, 360000);
        config.columns_per_packet = 16;
        config.lidar_mode = lidar_mode::MODE_1024x10;
        config.multipurpose_io_mode = MultipurposeIOMode::MULTIPURPOSE_OFF;
        config.nmea_baud_rate = NMEABaudRate::BAUD_9600;
        config.nmea_ignore_valid_char = false;
        config.nmea_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.nmea_leap_seconds = 0;
        config.phase_lock_enable = false;
        config.phase_lock_offset = 0;
        config.signal_multiplier = 1;
        config.sync_pulse_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_angle = 360;
        config.sync_pulse_out_frequency = 1;
        config.sync_pulse_out_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_pulse_width = 10;
        config.timestamp_mode = timestamp_mode::TIME_FROM_INTERNAL_OSC;
        config.udp_dest="192.168.88.254";
        config.udp_port_imu = 7503;
        config.udp_port_lidar = 7502; 
        config.udp_profile_imu = UDPProfileIMU::PROFILE_IMU_LEGACY;
        config.udp_profile_lidar = UDPProfileLidar::PROFILE_LIDAR_LEGACY;
        sinfo_populator(si_2_2_os_992119000444_128,
            "",
            992119000444,
            "v2.2.0-rc.1",
            MODE_1024x10,
            "OS-1-128",
            {128, 16, 1024, {24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0}, {0, 1023}, PROFILE_LIDAR_LEGACY, PROFILE_IMU_LEGACY, 10},
            {4.26, 1.44, -1.39, -4.22, 4.27, 1.45, -1.38, -4.22, 4.26, 1.45, -1.38, -4.2, 4.25, 1.43, -1.4, -4.22, 4.28, 1.44, -1.4, -4.22, 4.27, 1.45, -1.4, -4.22, 4.27, 1.44, -1.39, -4.24, 4.26, 1.43, -1.39, -4.24, 4.26, 1.44, -1.4, -4.24, 4.25, 1.4, -1.4, -4.25, 4.25, 1.44, -1.41, -4.25, 4.28, 1.42, -1.4, -4.24, 4.26, 1.41, -1.43, -4.25, 4.26, 1.41, -1.42, -4.24, 4.24, 1.43, -1.42, -4.26, 4.27, 1.42, -1.42, -4.27, 4.26, 1.41, -1.45, -4.24, 4.24, 1.42, -1.41, -4.24, 4.25, 1.42, -1.43, -4.25, 4.27, 1.39, -1.43, -4.26, 4.23, 1.39, -1.44, -4.27, 4.26, 1.41, -1.43, -4.26, 4.24, 1.4, -1.42, -4.28, 4.24, 1.42, -1.43, -4.26, 4.24, 1.4, -1.43, -4.26, 4.22, 1.4, -1.43, -4.27, 4.24, 1.41, -1.44, -4.28, 4.24, 1.4, -1.45, -4.29, 4.23, 1.39, -1.45, -4.27, 4.22, 1.38, -1.45, -4.27, 4.2, 1.39, -1.45, -4.29, 4.22, 1.39, -1.45, -4.29},
            {21.51, 21.18, 20.87, 20.57, 20.26, 19.94, 19.63, 19.32, 19.01, 18.68, 18.35, 18.05, 17.72, 17.38, 17.06, 16.74, 16.42, 16.09, 15.76, 15.44, 15.09, 14.77, 14.43, 14.1, 13.75, 13.42, 13.1, 12.77, 12.41, 12.06, 11.73, 11.4, 11.04, 10.71, 10.36, 10.03, 9.67, 9.32, 9.01, 8.65, 8.28, 7.94, 7.59, 7.26, 6.91, 6.53, 6.21, 5.87, 5.5, 5.15, 4.8, 4.46, 4.11, 3.74, 3.39, 3.06, 2.68, 2.34, 1.99, 1.62, 1.27, 0.92, 0.58, 0.21, -0.14, -0.5, -0.86, -1.18, -1.55, -1.91, -2.25, -2.59, -2.98, -3.33, -3.68, -4.01, -4.38, -4.74, -5.08, -5.44, -5.8, -6.14, -6.48, -6.83, -7.19, -7.54, -7.88, -8.22, -8.59, -8.94, -9.26, -9.63, -9.97, -10.3, -10.65, -10.97, -11.33, -11.68, -12.02, -12.36, -12.72, -13.06, -13.37, -13.7, -14.05, -14.39, -14.71, -15.06, -15.39, -15.72, -16.04, -16.38, -16.7, -17.03, -17.34, -17.67, -18.01, -18.33, -18.64, -18.93, -19.29, -19.59, -19.89, -20.2, -20.51, -20.82, -21.14, -21.44},
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1}),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1}),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            7109745,
            7502,
            7503,
            "2021-09-30T15:58:18Z",
            "ousteros-image-prod-aries-v2.2.0-rc.1",
            "840-102145-05",
            "RUNNING",
            calibration_status{true, "2021-05-14T20:45:35"},
            config
        );

        sinfo_populator(si_2_3_1_os_992146000760_128_legacy,
            "",
            992146000760,
            "v2.3.1-rc.1",
            MODE_1024x10,
            "OS-1-128",
            {128, 16, 1024, { 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0} , {0, 1023}, PROFILE_RNG15_RFL8_NIR8, PROFILE_IMU_LEGACY, 10},
            { 4.27, 1.43, -1.39, -4.23, 4.26, 1.45, -1.38, -4.22, 4.28, 1.43, -1.39, -4.24, 4.27, 1.44, -1.4, -4.22, 4.28, 1.45, -1.39, -4.23, 4.26, 1.44, -1.4, -4.23, 4.27, 1.43, -1.39, -4.23, 4.26, 1.43, -1.4, -4.24, 4.26, 1.44, -1.4, -4.23, 4.28, 1.43, -1.4, -4.24, 4.26, 1.42, -1.39, -4.24, 4.27, 1.43, -1.42, -4.25, 4.26, 1.43, -1.42, -4.25, 4.28, 1.43, -1.42, -4.25, 4.26, 1.43, -1.44, -4.25, 4.27, 1.43, -1.43, -4.26, 4.27, 1.42, -1.43, -4.26, 4.25, 1.42, -1.43, -4.28, 4.25, 1.42, -1.45, -4.28, 4.26, 1.42, -1.44, -4.26, 4.26, 1.4, -1.46, -4.27, 4.24, 1.41, -1.43, -4.28, 4.26, 1.4, -1.44, -4.28, 4.22, 1.4, -1.43, -4.29, 4.25, 1.41, -1.45, -4.29, 4.24, 1.4, -1.46, -4.28, 4.24, 1.4, -1.44, -4.28, 4.22, 1.41, -1.45, -4.28, 4.25, 1.39, -1.45, -4.29, 4.23, 1.4, -1.45, -4.3, 4.22, 1.39, -1.47, -4.29, 4.22, 1.38, -1.47, -4.3 },
            { 21.34, 21.03, 20.72, 20.41, 20.08, 19.78, 19.48, 19.17, 18.83, 18.51, 18.21, 17.88, 17.56, 17.23, 16.92, 16.59, 16.26, 15.94, 15.61, 15.28, 14.93, 14.6, 14.27, 13.95, 13.61, 13.26, 12.94, 12.6, 12.26, 11.91, 11.58, 11.24, 10.89, 10.55, 10.21, 9.88, 9.52, 9.18, 8.83, 8.5, 8.14, 7.78, 7.46, 7.11, 6.75, 6.39, 6.05, 5.7, 5.33, 4.99, 4.64, 4.29, 3.94, 3.59, 3.24, 2.88, 2.52, 2.18, 1.81, 1.48, 1.11, 0.76, 0.4, 0.05, -0.3, -0.67, -1.01, -1.37, -1.75, -2.08, -2.43, -2.79, -3.15, -3.5, -3.86, -4.2, -4.56, -4.9, -5.27, -5.6, -5.96, -6.32, -6.68, -7, -7.38, -7.71, -8.05, -8.41, -8.77, -9.11, -9.45, -9.79, -10.16, -10.49, -10.83, -11.18, -11.54, -11.86, -12.2, -12.55, -12.91, -13.24, -13.58, -13.89, -14.26, -14.58, -14.89, -15.21, -15.6, -15.89, -16.22, -16.54, -16.88, -17.21, -17.52, -17.85, -18.2, -18.5, -18.82, -19.14, -19.47, -19.78, -20.09, -20.38, -20.72, -21.03, -21.33, -21.62 },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({ 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1}),
            mkmat4d({ -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1}),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            5431287,
            7502,
            7503,
            "2022-05-25T19:18:50Z",
            "ousteros-image-dev-aries-v2.3.1-rc.1+20220525191702.patch-v2.3.x",
            "840-103575-06",
            "RUNNING",
            calibration_status{},
            sensor_config{}
        );

        config = sensor_config{};
        config.operating_mode = OperatingMode::OPERATING_NORMAL;
        config.azimuth_window = std::make_pair<int, int>(0, 360000);
        config.columns_per_packet = 16;
        config.lidar_mode = lidar_mode::MODE_1024x10;
        config.multipurpose_io_mode = MultipurposeIOMode::MULTIPURPOSE_OFF;
        config.nmea_baud_rate = NMEABaudRate::BAUD_9600;
        config.nmea_ignore_valid_char = false;
        config.nmea_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.nmea_leap_seconds = 0;
        config.phase_lock_enable = false;
        config.phase_lock_offset = 0;
        config.signal_multiplier = 1;
        config.sync_pulse_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_angle = 360;
        config.sync_pulse_out_frequency = 1;
        config.sync_pulse_out_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_pulse_width = 10;
        config.timestamp_mode = timestamp_mode::TIME_FROM_INTERNAL_OSC;
        config.udp_dest="192.168.1.80";
        config.udp_port_imu = 7503;
        config.udp_port_lidar = 7502; 
        config.udp_profile_imu = UDPProfileIMU::PROFILE_IMU_LEGACY;
        config.udp_profile_lidar = UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8;
        sinfo_populator( si_2_3_1_os_992146000760_128,
            "",
            992146000760,
            "v2.3.1-rc.1",
            MODE_1024x10,
            "OS-1-128",
            {128, 16, 1024, { 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0, 24, 16, 8, 0} , {0, 1023}, PROFILE_RNG15_RFL8_NIR8, PROFILE_IMU_LEGACY, 10},
            { 4.27, 1.43, -1.39, -4.23, 4.26, 1.45, -1.38, -4.22, 4.28, 1.43, -1.39, -4.24, 4.27, 1.44, -1.4, -4.22, 4.28, 1.45, -1.39, -4.23, 4.26, 1.44, -1.4, -4.23, 4.27, 1.43, -1.39, -4.23, 4.26, 1.43, -1.4, -4.24, 4.26, 1.44, -1.4, -4.23, 4.28, 1.43, -1.4, -4.24, 4.26, 1.42, -1.39, -4.24, 4.27, 1.43, -1.42, -4.25, 4.26, 1.43, -1.42, -4.25, 4.28, 1.43, -1.42, -4.25, 4.26, 1.43, -1.44, -4.25, 4.27, 1.43, -1.43, -4.26, 4.27, 1.42, -1.43, -4.26, 4.25, 1.42, -1.43, -4.28, 4.25, 1.42, -1.45, -4.28, 4.26, 1.42, -1.44, -4.26, 4.26, 1.4, -1.46, -4.27, 4.24, 1.41, -1.43, -4.28, 4.26, 1.4, -1.44, -4.28, 4.22, 1.4, -1.43, -4.29, 4.25, 1.41, -1.45, -4.29, 4.24, 1.4, -1.46, -4.28, 4.24, 1.4, -1.44, -4.28, 4.22, 1.41, -1.45, -4.28, 4.25, 1.39, -1.45, -4.29, 4.23, 1.4, -1.45, -4.3, 4.22, 1.39, -1.47, -4.29, 4.22, 1.38, -1.47, -4.3 },
            { 21.34, 21.03, 20.72, 20.41, 20.08, 19.78, 19.48, 19.17, 18.83, 18.51, 18.21, 17.88, 17.56, 17.23, 16.92, 16.59, 16.26, 15.94, 15.61, 15.28, 14.93, 14.6, 14.27, 13.95, 13.61, 13.26, 12.94, 12.6, 12.26, 11.91, 11.58, 11.24, 10.89, 10.55, 10.21, 9.88, 9.52, 9.18, 8.83, 8.5, 8.14, 7.78, 7.46, 7.11, 6.75, 6.39, 6.05, 5.7, 5.33, 4.99, 4.64, 4.29, 3.94, 3.59, 3.24, 2.88, 2.52, 2.18, 1.81, 1.48, 1.11, 0.76, 0.4, 0.05, -0.3, -0.67, -1.01, -1.37, -1.75, -2.08, -2.43, -2.79, -3.15, -3.5, -3.86, -4.2, -4.56, -4.9, -5.27, -5.6, -5.96, -6.32, -6.68, -7, -7.38, -7.71, -8.05, -8.41, -8.77, -9.11, -9.45, -9.79, -10.16, -10.49, -10.83, -11.18, -11.54, -11.86, -12.2, -12.55, -12.91, -13.24, -13.58, -13.89, -14.26, -14.58, -14.89, -15.21, -15.6, -15.89, -16.22, -16.54, -16.88, -17.21, -17.52, -17.85, -18.2, -18.5, -18.82, -19.14, -19.47, -19.78, -20.09, -20.38, -20.72, -21.03, -21.33, -21.62 },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({ 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1}),
            mkmat4d({ -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1}),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            5431287,
            7502,
            7503,
            "2022-05-25T19:18:50Z",
            "ousteros-image-dev-aries-v2.3.1-rc.1+20220525191702.patch-v2.3.x",
            "840-103575-06",
            "RUNNING",
            calibration_status{true, "2021-11-23T05:37:45"},
            config
        );

        config = sensor_config{};
        config.azimuth_window = std::make_pair<int, int>(0, 360000);
        config.columns_per_packet = 16;
        config.lidar_mode = lidar_mode::MODE_1024x10;
        config.multipurpose_io_mode = MultipurposeIOMode::MULTIPURPOSE_INPUT_NMEA_UART;
        config.nmea_baud_rate = NMEABaudRate::BAUD_115200;
        config.nmea_ignore_valid_char = false;
        config.nmea_in_polarity = Polarity::POLARITY_ACTIVE_LOW;
        config.nmea_leap_seconds = 0;
        config.operating_mode = OperatingMode::OPERATING_NORMAL;
        config.phase_lock_enable = false;
        config.phase_lock_offset = 0;
        config.signal_multiplier = 1;
        config.sync_pulse_in_polarity = Polarity::POLARITY_ACTIVE_LOW;
        config.sync_pulse_out_angle = 180;
        config.sync_pulse_out_frequency = 10;
        config.sync_pulse_out_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_pulse_width = 10;
        config.timestamp_mode = timestamp_mode::TIME_FROM_SYNC_PULSE_IN;
        config.udp_dest="10.0.0.167";
        config.udp_port_imu = 7503;
        config.udp_port_lidar = 7502; 
        config.udp_profile_imu = UDPProfileIMU::PROFILE_IMU_LEGACY;
        config.udp_profile_lidar = UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16;
        sinfo_populator(si_2_4_0_os_992146000760_128,
            "",
            992146000760,
            "v2.4.0",
            MODE_1024x10,
            "OS-1-128",
            {128, 16, 1024, { 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12} , {0, 1023}, PROFILE_RNG19_RFL8_SIG16_NIR16, PROFILE_IMU_LEGACY, 10},
            { 4.27, 1.43, -1.39, -4.23, 4.26, 1.45, -1.38, -4.22, 4.28, 1.43, -1.39, -4.24, 4.27, 1.44, -1.4, -4.22, 4.28, 1.45, -1.39, -4.23, 4.26, 1.44, -1.4, -4.23, 4.27, 1.43, -1.39, -4.23, 4.26, 1.43, -1.4, -4.24, 4.26, 1.44, -1.4, -4.23, 4.28, 1.43, -1.4, -4.24, 4.26, 1.42, -1.39, -4.24, 4.27, 1.43, -1.42, -4.25, 4.26, 1.43, -1.42, -4.25, 4.28, 1.43, -1.42, -4.25, 4.26, 1.43, -1.44, -4.25, 4.27, 1.43, -1.43, -4.26, 4.27, 1.42, -1.43, -4.26, 4.25, 1.42, -1.43, -4.28, 4.25, 1.42, -1.45, -4.28, 4.26, 1.42, -1.44, -4.26, 4.26, 1.4, -1.46, -4.27, 4.24, 1.41, -1.43, -4.28, 4.26, 1.4, -1.44, -4.28, 4.22, 1.4, -1.43, -4.29, 4.25, 1.41, -1.45, -4.29, 4.24, 1.4, -1.46, -4.28, 4.24, 1.4, -1.44, -4.28, 4.22, 1.41, -1.45, -4.28, 4.25, 1.39, -1.45, -4.29, 4.23, 1.4, -1.45, -4.3, 4.22, 1.39, -1.47, -4.29, 4.22, 1.38, -1.47, -4.3 },
            { 21.34, 21.03, 20.72, 20.41, 20.08, 19.78, 19.48, 19.17, 18.83, 18.51, 18.21, 17.88, 17.56, 17.23, 16.92, 16.59, 16.26, 15.94, 15.61, 15.28, 14.93, 14.6, 14.27, 13.95, 13.61, 13.26, 12.94, 12.6, 12.26, 11.91, 11.58, 11.24, 10.89, 10.55, 10.21, 9.88, 9.52, 9.18, 8.83, 8.5, 8.14, 7.78, 7.46, 7.11, 6.75, 6.39, 6.05, 5.7, 5.33, 4.99, 4.64, 4.29, 3.94, 3.59, 3.24, 2.88, 2.52, 2.18, 1.81, 1.48, 1.11, 0.76, 0.4, 0.05, -0.3, -0.67, -1.01, -1.37, -1.75, -2.08, -2.43, -2.79, -3.15, -3.5, -3.86, -4.2, -4.56, -4.9, -5.27, -5.6, -5.96, -6.32, -6.68, -7, -7.38, -7.71, -8.05, -8.41, -8.77, -9.11, -9.45, -9.79, -10.16, -10.49, -10.83, -11.18, -11.54, -11.86, -12.2, -12.55, -12.91, -13.24, -13.58, -13.89, -14.26, -14.58, -14.89, -15.21, -15.6, -15.89, -16.22, -16.54, -16.88, -17.21, -17.52, -17.85, -18.2, -18.5, -18.82, -19.14, -19.47, -19.78, -20.09, -20.38, -20.72, -21.03, -21.33, -21.62 },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({ 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1}),
            mkmat4d({ -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1}),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            2573180,
            7502,
            7503,
            "2022-09-21T17:47:45Z",
            "ousteros-image-prod-aries-v2.4.0+20220921174636",
            "840-103575-06",
            "RUNNING",
            calibration_status{true, "2021-11-23T05:37:45"},
            config
        );

        sinfo_populator(si_2_4_0_os_992146000760_128_legacy,
            "",
            992146000760,
            "v2.4.0",
            MODE_1024x10,
            "OS-1-128",
            {128, 16, 1024, { 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12} , {0, 1023}, PROFILE_RNG19_RFL8_SIG16_NIR16, PROFILE_IMU_LEGACY, 10},
            { 4.27, 1.43, -1.39, -4.23, 4.26, 1.45, -1.38, -4.22, 4.28, 1.43, -1.39, -4.24, 4.27, 1.44, -1.4, -4.22, 4.28, 1.45, -1.39, -4.23, 4.26, 1.44, -1.4, -4.23, 4.27, 1.43, -1.39, -4.23, 4.26, 1.43, -1.4, -4.24, 4.26, 1.44, -1.4, -4.23, 4.28, 1.43, -1.4, -4.24, 4.26, 1.42, -1.39, -4.24, 4.27, 1.43, -1.42, -4.25, 4.26, 1.43, -1.42, -4.25, 4.28, 1.43, -1.42, -4.25, 4.26, 1.43, -1.44, -4.25, 4.27, 1.43, -1.43, -4.26, 4.27, 1.42, -1.43, -4.26, 4.25, 1.42, -1.43, -4.28, 4.25, 1.42, -1.45, -4.28, 4.26, 1.42, -1.44, -4.26, 4.26, 1.4, -1.46, -4.27, 4.24, 1.41, -1.43, -4.28, 4.26, 1.4, -1.44, -4.28, 4.22, 1.4, -1.43, -4.29, 4.25, 1.41, -1.45, -4.29, 4.24, 1.4, -1.46, -4.28, 4.24, 1.4, -1.44, -4.28, 4.22, 1.41, -1.45, -4.28, 4.25, 1.39, -1.45, -4.29, 4.23, 1.4, -1.45, -4.3, 4.22, 1.39, -1.47, -4.29, 4.22, 1.38, -1.47, -4.3 },
            { 21.34, 21.03, 20.72, 20.41, 20.08, 19.78, 19.48, 19.17, 18.83, 18.51, 18.21, 17.88, 17.56, 17.23, 16.92, 16.59, 16.26, 15.94, 15.61, 15.28, 14.93, 14.6, 14.27, 13.95, 13.61, 13.26, 12.94, 12.6, 12.26, 11.91, 11.58, 11.24, 10.89, 10.55, 10.21, 9.88, 9.52, 9.18, 8.83, 8.5, 8.14, 7.78, 7.46, 7.11, 6.75, 6.39, 6.05, 5.7, 5.33, 4.99, 4.64, 4.29, 3.94, 3.59, 3.24, 2.88, 2.52, 2.18, 1.81, 1.48, 1.11, 0.76, 0.4, 0.05, -0.3, -0.67, -1.01, -1.37, -1.75, -2.08, -2.43, -2.79, -3.15, -3.5, -3.86, -4.2, -4.56, -4.9, -5.27, -5.6, -5.96, -6.32, -6.68, -7, -7.38, -7.71, -8.05, -8.41, -8.77, -9.11, -9.45, -9.79, -10.16, -10.49, -10.83, -11.18, -11.54, -11.86, -12.2, -12.55, -12.91, -13.24, -13.58, -13.89, -14.26, -14.58, -14.89, -15.21, -15.6, -15.89, -16.22, -16.54, -16.88, -17.21, -17.52, -17.85, -18.2, -18.5, -18.82, -19.14, -19.47, -19.78, -20.09, -20.38, -20.72, -21.03, -21.33, -21.62 },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({ 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1}),
            mkmat4d({ -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1}),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            2573180,
            7502,
            7503,
            "2022-09-21T17:47:45Z",
            "ousteros-image-prod-aries-v2.4.0+20220921174636",
            "840-103575-06",
            "RUNNING",
            calibration_status{},
            sensor_config{}
        );

        sinfo_populator( si_3_0_1_os_122246000293_128_legacy,
            "",
            122246000293,
            "v3.0.1-rc.1",
            MODE_1024x10,
            "OS-1-128",
            {128, 16, 1024, { 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12}, {0, 1023}, PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL, PROFILE_IMU_LEGACY, 10},
            { 4.2, 1.42, -1.37, -4.14, 4.22, 1.44, -1.37, -4.15, 4.24, 1.43, -1.36, -4.15, 4.24, 1.43, -1.36, -4.16, 4.23, 1.42, -1.37, -4.17, 4.25, 1.43, -1.37, -4.16, 4.24, 1.42, -1.38, -4.18, 4.24, 1.42, -1.38, -4.18, 4.24, 1.41, -1.38, -4.18, 4.23, 1.41, -1.39, -4.19, 4.22, 1.4, -1.39, -4.2, 4.21, 1.4, -1.4, -4.2, 4.2, 1.41, -1.4, -4.21, 4.2, 1.4, -1.41, -4.21, 4.2, 1.4, -1.41, -4.21, 4.2, 1.4, -1.42, -4.21, 4.2, 1.4, -1.42, -4.22, 4.2, 1.39, -1.41, -4.22, 4.2, 1.38, -1.41, -4.23, 4.2, 1.38, -1.41, -4.24, 4.2, 1.39, -1.42, -4.23, 4.2, 1.38, -1.43, -4.22, 4.2, 1.38, -1.42, -4.23, 4.19, 1.38, -1.42, -4.23, 4.2, 1.39, -1.43, -4.22, 4.2, 1.38, -1.42, -4.23, 4.19, 1.39, -1.43, -4.24, 4.19, 1.38, -1.43, -4.23, 4.2, 1.38, -1.42, -4.24, 4.2, 1.38, -1.43, -4.24, 4.19, 1.37, -1.43, -4.24, 4.19, 1.37, -1.44, -4.25},
            { 20.47, 20.21, 19.89, 19.54, 19.23, 18.97, 18.66, 18.31, 18, 17.72, 17.4, 17.05, 16.74, 16.45, 16.13, 15.77, 15.45, 15.15, 14.83, 14.47, 14.15, 13.85, 13.52, 13.17, 12.83, 12.52, 12.19, 11.83, 11.5, 11.18, 10.85, 10.5, 10.14, 9.82, 9.49, 9.14, 8.79, 8.45, 8.13, 7.76, 7.42, 7.08, 6.75, 6.39, 6.04, 5.7, 5.35, 5, 4.64, 4.32, 3.98, 3.61, 3.25, 2.92, 2.57, 2.22, 1.85, 1.52, 1.17, 0.82, 0.45, 0.12, -0.24, -0.58, -0.95, -1.28, -1.64, -1.97, -2.35, -2.69, -3.03, -3.37, -3.74, -4.1, -4.43, -4.77, -5.14, -5.49, -5.82, -6.17, -6.53, -6.88, -7.22, -7.55, -7.91, -8.27, -8.6, -8.92, -9.28, -9.64, -9.97, -10.29, -10.65, -11.01, -11.33, -11.64, -12.01, -12.35, -12.69, -12.98, -13.34, -13.69, -14.03, -14.32, -14.68, -15.02, -15.35, -15.63, -15.98, -16.34, -16.65, -16.93, -17.27, -17.63, -17.93, -18.21, -18.55, -18.9, -19.2, -19.48, -19.81, -20.16, -20.46, -20.72, -21.05, -21.39, -21.68, -21.95},
            16.721,
            mkmat4d({ 1, 0, 0, 16.721, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}),
            mkmat4d({1, 0, 0, -2.441, 0, 1, 0, -9.725, 0, 0, 1, 7.533, 0, 0, 0, 1}),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 38.195, 0, 0, 0, 1}),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            1629346,
            7502,
            7503,
            "2023-02-03T21:45:40Z",
            "ousteros-image-prod-bootes-v3.0.1-rc.1+20230203215650.staging-2022",
            "840-104682-C",
            "RUNNING",
            calibration_status{},
            sensor_config{}
        );

        config = sensor_config{};
        config.azimuth_window = std::make_pair<int, int>(0, 360000);
        config.columns_per_packet = 16;
        config.lidar_mode = lidar_mode::MODE_1024x10;
        config.multipurpose_io_mode = MultipurposeIOMode::MULTIPURPOSE_OFF;
        config.nmea_baud_rate = NMEABaudRate::BAUD_9600;
        config.nmea_ignore_valid_char = false;
        config.nmea_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.nmea_leap_seconds = 0;
        config.operating_mode = OperatingMode::OPERATING_NORMAL;
        config.phase_lock_enable = false;
        config.phase_lock_offset = 0;
        config.signal_multiplier = 1;
        config.sync_pulse_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_angle = 360;
        config.sync_pulse_out_frequency = 1;
        config.sync_pulse_out_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_pulse_width = 10;
        config.timestamp_mode = timestamp_mode::TIME_FROM_INTERNAL_OSC;
        config.udp_dest="10.0.0.167";
        config.udp_port_imu = 7503;
        config.udp_port_lidar = 7502; 
        config.udp_profile_imu = UDPProfileIMU::PROFILE_IMU_LEGACY;
        config.udp_profile_lidar = UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL;
        sinfo_populator(si_3_0_1_os_122246000293_128,
            "",
            122246000293,
            "v3.0.1-rc.1",
            MODE_1024x10,
            "OS-1-128",
            {128, 16, 1024, { 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12}, {0, 1023}, PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL, PROFILE_IMU_LEGACY, 10},
            { 4.2, 1.42, -1.37, -4.14, 4.22, 1.44, -1.37, -4.15, 4.24, 1.43, -1.36, -4.15, 4.24, 1.43, -1.36, -4.16, 4.23, 1.42, -1.37, -4.17, 4.25, 1.43, -1.37, -4.16, 4.24, 1.42, -1.38, -4.18, 4.24, 1.42, -1.38, -4.18, 4.24, 1.41, -1.38, -4.18, 4.23, 1.41, -1.39, -4.19, 4.22, 1.4, -1.39, -4.2, 4.21, 1.4, -1.4, -4.2, 4.2, 1.41, -1.4, -4.21, 4.2, 1.4, -1.41, -4.21, 4.2, 1.4, -1.41, -4.21, 4.2, 1.4, -1.42, -4.21, 4.2, 1.4, -1.42, -4.22, 4.2, 1.39, -1.41, -4.22, 4.2, 1.38, -1.41, -4.23, 4.2, 1.38, -1.41, -4.24, 4.2, 1.39, -1.42, -4.23, 4.2, 1.38, -1.43, -4.22, 4.2, 1.38, -1.42, -4.23, 4.19, 1.38, -1.42, -4.23, 4.2, 1.39, -1.43, -4.22, 4.2, 1.38, -1.42, -4.23, 4.19, 1.39, -1.43, -4.24, 4.19, 1.38, -1.43, -4.23, 4.2, 1.38, -1.42, -4.24, 4.2, 1.38, -1.43, -4.24, 4.19, 1.37, -1.43, -4.24, 4.19, 1.37, -1.44, -4.25},
            { 20.47, 20.21, 19.89, 19.54, 19.23, 18.97, 18.66, 18.31, 18, 17.72, 17.4, 17.05, 16.74, 16.45, 16.13, 15.77, 15.45, 15.15, 14.83, 14.47, 14.15, 13.85, 13.52, 13.17, 12.83, 12.52, 12.19, 11.83, 11.5, 11.18, 10.85, 10.5, 10.14, 9.82, 9.49, 9.14, 8.79, 8.45, 8.13, 7.76, 7.42, 7.08, 6.75, 6.39, 6.04, 5.7, 5.35, 5, 4.64, 4.32, 3.98, 3.61, 3.25, 2.92, 2.57, 2.22, 1.85, 1.52, 1.17, 0.82, 0.45, 0.12, -0.24, -0.58, -0.95, -1.28, -1.64, -1.97, -2.35, -2.69, -3.03, -3.37, -3.74, -4.1, -4.43, -4.77, -5.14, -5.49, -5.82, -6.17, -6.53, -6.88, -7.22, -7.55, -7.91, -8.27, -8.6, -8.92, -9.28, -9.64, -9.97, -10.29, -10.65, -11.01, -11.33, -11.64, -12.01, -12.35, -12.69, -12.98, -13.34, -13.69, -14.03, -14.32, -14.68, -15.02, -15.35, -15.63, -15.98, -16.34, -16.65, -16.93, -17.27, -17.63, -17.93, -18.21, -18.55, -18.9, -19.2, -19.48, -19.81, -20.16, -20.46, -20.72, -21.05, -21.39, -21.68, -21.95},
            16.721,
            mkmat4d({ 1, 0, 0, 16.721, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}),
            mkmat4d({1, 0, 0, -2.441, 0, 1, 0, -9.725, 0, 0, 1, 7.533, 0, 0, 0, 1}),
            mkmat4d({-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 38.195, 0, 0, 0, 1}),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            1629346,
            7502,
            7503,
            "2023-02-03T21:45:40Z",
            "ousteros-image-prod-bootes-v3.0.1-rc.1+20230203215650.staging-2022",
            "840-104682-C",
            "RUNNING",
            calibration_status{true, "2022-11-29T02:00:08"},
            config
        );

        sinfo_populator( si_2_5_0_os_992146000760_128_legacy,
            "",
            992146000760,
            "v2.5.0-omega.8",
            MODE_1024x10,
            "OS-1-128",
            {128, 16, 1024, { 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12} , {912, 1022}, PROFILE_RNG19_RFL8_SIG16_NIR16, PROFILE_IMU_LEGACY, 10},
            { 4.27, 1.43, -1.39, -4.23, 4.26, 1.45, -1.38, -4.22, 4.28, 1.43, -1.39, -4.24, 4.27, 1.44, -1.4, -4.22, 4.28, 1.45, -1.39, -4.23, 4.26, 1.44, -1.4, -4.23, 4.27, 1.43, -1.39, -4.23, 4.26, 1.43, -1.4, -4.24, 4.26, 1.44, -1.4, -4.23, 4.28, 1.43, -1.4, -4.24, 4.26, 1.42, -1.39, -4.24, 4.27, 1.43, -1.42, -4.25, 4.26, 1.43, -1.42, -4.25, 4.28, 1.43, -1.42, -4.25, 4.26, 1.43, -1.44, -4.25, 4.27, 1.43, -1.43, -4.26, 4.27, 1.42, -1.43, -4.26, 4.25, 1.42, -1.43, -4.28, 4.25, 1.42, -1.45, -4.28, 4.26, 1.42, -1.44, -4.26, 4.26, 1.4, -1.46, -4.27, 4.24, 1.41, -1.43, -4.28, 4.26, 1.4, -1.44, -4.28, 4.22, 1.4, -1.43, -4.29, 4.25, 1.41, -1.45, -4.29, 4.24, 1.4, -1.46, -4.28, 4.24, 1.4, -1.44, -4.28, 4.22, 1.41, -1.45, -4.28, 4.25, 1.39, -1.45, -4.29, 4.23, 1.4, -1.45, -4.3, 4.22, 1.39, -1.47, -4.29, 4.22, 1.38, -1.47, -4.3 },
            { 21.34, 21.03, 20.72, 20.41, 20.08, 19.78, 19.48, 19.17, 18.83, 18.51, 18.21, 17.88, 17.56, 17.23, 16.92, 16.59, 16.26, 15.94, 15.61, 15.28, 14.93, 14.6, 14.27, 13.95, 13.61, 13.26, 12.94, 12.6, 12.26, 11.91, 11.58, 11.24, 10.89, 10.55, 10.21, 9.88, 9.52, 9.18, 8.83, 8.5, 8.14, 7.78, 7.46, 7.11, 6.75, 6.39, 6.05, 5.7, 5.33, 4.99, 4.64, 4.29, 3.94, 3.59, 3.24, 2.88, 2.52, 2.18, 1.81, 1.48, 1.11, 0.76, 0.4, 0.05, -0.3, -0.67, -1.01, -1.37, -1.75, -2.08, -2.43, -2.79, -3.15, -3.5, -3.86, -4.2, -4.56, -4.9, -5.27, -5.6, -5.96, -6.32, -6.68, -7, -7.38, -7.71, -8.05, -8.41, -8.77, -9.11, -9.45, -9.79, -10.16, -10.49, -10.83, -11.18, -11.54, -11.86, -12.2, -12.55, -12.91, -13.24, -13.58, -13.89, -14.26, -14.58, -14.89, -15.21, -15.6, -15.89, -16.22, -16.54, -16.88, -17.21, -17.52, -17.85, -18.2, -18.5, -18.82, -19.14, -19.47, -19.78, -20.09, -20.38, -20.72, -21.03, -21.33, -21.62 },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({ 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1}),
            mkmat4d({ -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1}),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            8247301,
            7502,
            7503,
            "2023-03-18T18:28:26Z",
            "ousteros-image-prod-aries-v2.5.0-omega.8+20230318213158.staging",
            "840-103575-06",
            "RUNNING",
            calibration_status(),
            sensor_config{}
        );

        config = sensor_config{};
        config.azimuth_window = std::make_pair<int, int>(583, 39402);
        config.columns_per_packet = 16;
        config.lidar_mode = lidar_mode::MODE_1024x10;
        config.multipurpose_io_mode = MultipurposeIOMode::MULTIPURPOSE_OFF;
        config.nmea_baud_rate = NMEABaudRate::BAUD_9600;
        config.nmea_ignore_valid_char = false;
        config.nmea_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.nmea_leap_seconds = 0;
        config.operating_mode = OperatingMode::OPERATING_NORMAL;
        config.phase_lock_enable = false;
        config.phase_lock_offset = 0;
        config.signal_multiplier = 1;
        config.sync_pulse_in_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_angle = 360;
        config.sync_pulse_out_frequency = 1;
        config.sync_pulse_out_polarity = Polarity::POLARITY_ACTIVE_HIGH;
        config.sync_pulse_out_pulse_width = 10;
        config.timestamp_mode = timestamp_mode::TIME_FROM_INTERNAL_OSC;
        config.udp_dest="10.0.0.167";
        config.udp_port_imu = 7503;
        config.udp_port_lidar = 7502; 
        config.udp_profile_imu = UDPProfileIMU::PROFILE_IMU_LEGACY;
        config.udp_profile_lidar = UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16;
        sinfo_populator(si_2_5_0_os_992146000760_128,
            "",
            992146000760,
            "v2.5.0-omega.8",
            MODE_1024x10,
            "OS-1-128",
            {128, 16, 1024, { 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12, 12, 4, -4, -12} , {912, 1022}, PROFILE_RNG19_RFL8_SIG16_NIR16, PROFILE_IMU_LEGACY, 10},
            { 4.27, 1.43, -1.39, -4.23, 4.26, 1.45, -1.38, -4.22, 4.28, 1.43, -1.39, -4.24, 4.27, 1.44, -1.4, -4.22, 4.28, 1.45, -1.39, -4.23, 4.26, 1.44, -1.4, -4.23, 4.27, 1.43, -1.39, -4.23, 4.26, 1.43, -1.4, -4.24, 4.26, 1.44, -1.4, -4.23, 4.28, 1.43, -1.4, -4.24, 4.26, 1.42, -1.39, -4.24, 4.27, 1.43, -1.42, -4.25, 4.26, 1.43, -1.42, -4.25, 4.28, 1.43, -1.42, -4.25, 4.26, 1.43, -1.44, -4.25, 4.27, 1.43, -1.43, -4.26, 4.27, 1.42, -1.43, -4.26, 4.25, 1.42, -1.43, -4.28, 4.25, 1.42, -1.45, -4.28, 4.26, 1.42, -1.44, -4.26, 4.26, 1.4, -1.46, -4.27, 4.24, 1.41, -1.43, -4.28, 4.26, 1.4, -1.44, -4.28, 4.22, 1.4, -1.43, -4.29, 4.25, 1.41, -1.45, -4.29, 4.24, 1.4, -1.46, -4.28, 4.24, 1.4, -1.44, -4.28, 4.22, 1.41, -1.45, -4.28, 4.25, 1.39, -1.45, -4.29, 4.23, 1.4, -1.45, -4.3, 4.22, 1.39, -1.47, -4.29, 4.22, 1.38, -1.47, -4.3 },
            { 21.34, 21.03, 20.72, 20.41, 20.08, 19.78, 19.48, 19.17, 18.83, 18.51, 18.21, 17.88, 17.56, 17.23, 16.92, 16.59, 16.26, 15.94, 15.61, 15.28, 14.93, 14.6, 14.27, 13.95, 13.61, 13.26, 12.94, 12.6, 12.26, 11.91, 11.58, 11.24, 10.89, 10.55, 10.21, 9.88, 9.52, 9.18, 8.83, 8.5, 8.14, 7.78, 7.46, 7.11, 6.75, 6.39, 6.05, 5.7, 5.33, 4.99, 4.64, 4.29, 3.94, 3.59, 3.24, 2.88, 2.52, 2.18, 1.81, 1.48, 1.11, 0.76, 0.4, 0.05, -0.3, -0.67, -1.01, -1.37, -1.75, -2.08, -2.43, -2.79, -3.15, -3.5, -3.86, -4.2, -4.56, -4.9, -5.27, -5.6, -5.96, -6.32, -6.68, -7, -7.38, -7.71, -8.05, -8.41, -8.77, -9.11, -9.45, -9.79, -10.16, -10.49, -10.83, -11.18, -11.54, -11.86, -12.2, -12.55, -12.91, -13.24, -13.58, -13.89, -14.26, -14.58, -14.89, -15.21, -15.6, -15.89, -16.22, -16.54, -16.88, -17.21, -17.52, -17.85, -18.2, -18.5, -18.82, -19.14, -19.47, -19.78, -20.09, -20.38, -20.72, -21.03, -21.33, -21.62 },
            15.806,
            mkmat4d({1, 0, 0, 15.806, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,}),
            mkmat4d({ 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1}),
            mkmat4d({ -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1}),
            mkmat4d({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, }),
            8247301,
            7502,
            7503,
            "2023-03-18T18:28:26Z",
            "ousteros-image-prod-aries-v2.5.0-omega.8+20230318213158.staging",
            "840-103575-06",
            "RUNNING",
            calibration_status{true, "2021-11-23T05:37:45"},
            config
        );
    }
};

INSTANTIATE_TEST_CASE_P(BCompat, MetaJsonTest,
testing::Values(
  "1_12_os1-991913000010-64",
  "1_12_os1-991937000062-64_legacy",
  "1_13_os1-991913000010-64",
  "1_14_6cccd_os-882002000138-128_legacy",
  "1_14_6cccd_os-882002000138-32U0_legacy",
  "1_14_beta_os1-991937000062-16A0_legacy",
  "1_14_beta_os1-991937000062-64_legacy",
  "ouster-studio-reduced-config-v1",
  "2_0_rc2_os-992011000121-32U0_legacy",
  "2_0_0_os1-992008000494-128_col_win_legacy",
  "2_0_0_os1-991913000010-64",
  "2_1_2_os1-991913000010-64_legacy",
  "2_1_2_os1-991913000010-64",
  "2_2_os-992119000444-128_legacy",
  "2_2_os-992119000444-128",
  "2_3_1_os-992146000760-128_legacy",
  "2_3_1_os-992146000760-128",
  "2_4_0_os-992146000760-128_legacy",
  "2_4_0_os-992146000760-128",
  "2_5_0_os-992146000760-128_legacy",
  "2_5_0_os-992146000760-128",
  "3_0_1_os-122246000293-128_legacy",
  "3_0_1_os-122246000293-128"
 ));

// clang-format on

std::string data_dir = ".";

// Backwards-compatibility test for meta json parsing: compare previously
// parsed sensor_info structs to the output of metadata_from_json
TEST_P(MetaJsonTest, MetadataFromJson) {
    std::string param = GetParam();

    auto data_dir = getenvs("DATA_DIR");

    // parse json file
    sensor_info si = metadata_from_json(data_dir + "/" + param + ".json");

    // compare with previously parsed struct from bcompat_sensor_info_data.h
    const sensor_info si_expected = *(expected_sensor_infos.at(param));

    EXPECT_EQ(si.sn, si_expected.sn);
    EXPECT_EQ(si.fw_rev, si_expected.fw_rev);
    EXPECT_EQ(si.config.lidar_mode, si_expected.config.lidar_mode);
    EXPECT_EQ(si.prod_line, si_expected.prod_line);

    EXPECT_EQ(si.format.pixels_per_column,
              si_expected.format.pixels_per_column);
    EXPECT_EQ(si.format.columns_per_packet,
              si_expected.format.columns_per_packet);
    EXPECT_EQ(si.format.columns_per_frame,
              si_expected.format.columns_per_frame);
    EXPECT_EQ(si.format.pixel_shift_by_row,
              si_expected.format.pixel_shift_by_row);
    EXPECT_EQ(si.format.column_window, si_expected.format.column_window);

    EXPECT_EQ(si.beam_azimuth_angles, si_expected.beam_azimuth_angles);
    EXPECT_EQ(si.beam_altitude_angles, si_expected.beam_altitude_angles);
    EXPECT_EQ(si.lidar_origin_to_beam_origin_mm,
              si_expected.lidar_origin_to_beam_origin_mm);
    EXPECT_EQ(si.beam_to_lidar_transform, si_expected.beam_to_lidar_transform);

    EXPECT_EQ(si.imu_to_sensor_transform, si_expected.imu_to_sensor_transform);
    EXPECT_EQ(si.lidar_to_sensor_transform,
              si_expected.lidar_to_sensor_transform);
    EXPECT_EQ(si.extrinsic, si_expected.extrinsic);
    EXPECT_EQ(si.init_id, si_expected.init_id);
    EXPECT_EQ(si.config.udp_port_lidar, si_expected.config.udp_port_lidar);
    EXPECT_EQ(si.config.udp_port_imu, si_expected.config.udp_port_imu);
    EXPECT_EQ(si.build_date, si_expected.build_date);
    EXPECT_EQ(si.image_rev, si_expected.image_rev);
    EXPECT_EQ(si.prod_pn, si_expected.prod_pn);
    EXPECT_EQ(si.status, si_expected.status);
    if (si.cal != si_expected.cal) {
        std::cout << param << std::endl;
        std::cout << "Expected: " << to_string(si_expected.cal) << std::endl;
        std::cout << "Actual: " << to_string(si.cal) << std::endl;
    }
    EXPECT_EQ(si.cal, si_expected.cal);
    if (si.config != si_expected.config) {
        std::cout << param << std::endl;
        std::cout << "Expected: " << to_string(si_expected.config) << std::endl;
        std::cout << "Actual: " << to_string(si.config) << std::endl;
    }
    EXPECT_EQ(si.config, si_expected.config);

    EXPECT_TRUE(si == si_expected);
    EXPECT_TRUE(si.has_fields_equal(si_expected));  // but the rest is the same
}

std::string my_replace(std::string source, const std::string& target,
                       const std::string& replacement) {
    size_t start_position = 0;
    while ((start_position = source.find(target, start_position)) !=
           std::string::npos) {
        source.replace(start_position, target.length(), replacement);
        start_position += replacement.length();
    }
    return source;
}

// Use this to make a text file that you copy and paste into the
// expected_issues map at the top
void make_expected(const std::string& path,
                   const std::vector<std::string>& actual_issues,
                   const std::string& param) {
    std::ofstream outfile;

    outfile.open(path,
                 std::ios_base::app);  // append instead of overwrite

    outfile << "{\"" << param << "\", {";

    int total = actual_issues.size();
    for (int i = 0; i < total; i++) {
        std::string out = actual_issues[i];
        outfile << "\"" << my_replace(out, "\"", "\\\"") << "\"";
        if (i != (total - 1)) {
            outfile << ",";
        }
    }
    outfile << "}},";
    outfile << std::endl;
    outfile.close();
}

// Backwards-compatibility test for meta json parsing: compare previously
// parsed sensor_info structs to the output of metadata_from_json
TEST_P(MetaJsonTest, MetadataErrorTests) {
    std::string param = GetParam();

    auto data_dir = getenvs("DATA_DIR");

    // parse json file
    nonstd::optional<ouster::sensor::sensor_info> sensor_info;
    ouster::ValidatorIssues issues;

    std::ifstream ifs;
    ifs.open(data_dir + "/" + param + ".json", std::ifstream::in);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    ouster::parse_and_validate_metadata(buffer.str(), sensor_info, issues);

    std::vector<std::string> actual_issues;
    for (auto it : issues.critical) {
        actual_issues.push_back(it.get_path());
    }
    for (auto it : issues.warning) {
        actual_issues.push_back(it.get_path());
    }
    for (auto it : issues.information) {
        actual_issues.push_back(it.get_path());
    }
    EXPECT_EQ(actual_issues, expected_issues.at(param));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
