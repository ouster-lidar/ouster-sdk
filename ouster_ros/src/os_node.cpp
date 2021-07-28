/**
 * @file
 * @brief Example node to publish raw sensor output on ROS topics
 *
 * ROS Parameters
 * sensor_hostname: hostname or IP in dotted decimal form of the sensor
 * udp_dest: hostname or IP where the sensor will send data packets
 * lidar_port: port to which the sensor should send lidar data
 * imu_port: port to which the sensor should send imu data
 */

#include <ros/console.h>
#include <ros/ros.h>

#include <fstream>
#include <sstream>
#include <string>

#include "ouster/build.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

using PacketMsg = ouster_ros::PacketMsg;
using OSConfigSrv = ouster_ros::OSConfigSrv;
namespace sensor = ouster::sensor;

// fill in values that could not be parsed from metadata
void populate_metadata_defaults(sensor::sensor_info& info,
                                sensor::lidar_mode specified_lidar_mode) {
    if (!info.name.size()) info.name = "UNKNOWN";

    if (!info.sn.size()) info.sn = "UNKNOWN";

    ouster::util::version v = ouster::util::version_of_string(info.fw_rev);
    if (v == ouster::util::invalid_version)
        ROS_WARN("Unknown sensor firmware version; output may not be reliable");
    else if (v < sensor::min_version)
        ROS_WARN("Firmware < %s not supported; output may not be reliable",
                 to_string(sensor::min_version).c_str());

    if (!info.mode) {
        ROS_WARN(
            "Lidar mode not found in metadata; output may not be reliable");
        info.mode = specified_lidar_mode;
    }

    if (!info.prod_line.size()) info.prod_line = "UNKNOWN";

    if (info.beam_azimuth_angles.empty() || info.beam_altitude_angles.empty()) {
        ROS_WARN("Beam angles not found in metadata; using design values");
        info.beam_azimuth_angles = sensor::gen1_azimuth_angles;
        info.beam_altitude_angles = sensor::gen1_altitude_angles;
    }
}

// try to write metadata file
bool write_metadata(const std::string& meta_file, const std::string& metadata) {
    std::ofstream ofs;
    ofs.open(meta_file);
    ofs << metadata << std::endl;
    ofs.close();
    if (ofs) {
        ROS_INFO("Wrote metadata to %s", meta_file.c_str());
    } else {
        ROS_WARN(
            "Failed to write metadata to %s; check that the path is valid. If "
            "you provided a relative path, please note that the working "
            "directory of all ROS nodes is set by default to $ROS_HOME",
            meta_file.c_str());
        return false;
    }
    return true;
}

int connection_loop(ros::NodeHandle& nh, sensor::client& cli,
                    const sensor::sensor_info& info) {
    auto lidar_packet_pub = nh.advertise<PacketMsg>("lidar_packets", 1280);
    auto imu_packet_pub = nh.advertise<PacketMsg>("imu_packets", 100);

    auto pf = sensor::get_format(info);

    PacketMsg lidar_packet, imu_packet;
    lidar_packet.buf.resize(pf.lidar_packet_size + 1);
    imu_packet.buf.resize(pf.imu_packet_size + 1);

    while (ros::ok()) {
        auto state = sensor::poll_client(cli);
        if (state == sensor::EXIT) {
            ROS_INFO("poll_client: caught signal, exiting");
            return EXIT_SUCCESS;
        }
        if (state & sensor::CLIENT_ERROR) {
            ROS_ERROR("poll_client: returned error");
            return EXIT_FAILURE;
        }
        if (state & sensor::LIDAR_DATA) {
            if (sensor::read_lidar_packet(cli, lidar_packet.buf.data(), pf))
                lidar_packet_pub.publish(lidar_packet);
        }
        if (state & sensor::IMU_DATA) {
            if (sensor::read_imu_packet(cli, imu_packet.buf.data(), pf))
                imu_packet_pub.publish(imu_packet);
        }
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "os_node");
    ros::NodeHandle nh("~");

    std::string published_metadata;
    auto srv = nh.advertiseService<OSConfigSrv::Request, OSConfigSrv::Response>(
        "os_config", [&](OSConfigSrv::Request&, OSConfigSrv::Response& res) {
            if (published_metadata.size()) {
                res.metadata = published_metadata;
                return true;
            } else
                return false;
        });

    // empty indicates "not set" since roslaunch xml can't optionally set params
    auto hostname = nh.param("sensor_hostname", std::string{});
    auto udp_dest = nh.param("udp_dest", std::string{});
    auto lidar_port = nh.param("lidar_port", 0);
    auto imu_port = nh.param("imu_port", 0);
    auto replay = nh.param("replay", false);
    auto lidar_mode_arg = nh.param("lidar_mode", std::string{});
    auto timestamp_mode_arg = nh.param("timestamp_mode", std::string{});

    // set lidar mode from param
    sensor::lidar_mode lidar_mode = sensor::MODE_UNSPEC;
    if (lidar_mode_arg.size()) {
        if (replay) ROS_WARN("Lidar mode set in replay mode. May be ignored");

        lidar_mode = sensor::lidar_mode_of_string(lidar_mode_arg);
        if (!lidar_mode) {
            ROS_ERROR("Invalid lidar mode %s", lidar_mode_arg.c_str());
            return EXIT_FAILURE;
        }
    }

    // set timestamp mode from param
    sensor::timestamp_mode timestamp_mode = sensor::TIME_FROM_UNSPEC;
    if (timestamp_mode_arg.size()) {
        if (replay)
            ROS_WARN("Timestamp mode set in replay mode. Will be ignored");

        timestamp_mode = sensor::timestamp_mode_of_string(timestamp_mode_arg);
        if (!timestamp_mode) {
            ROS_ERROR("Invalid timestamp mode %s", timestamp_mode_arg.c_str());
            return EXIT_FAILURE;
        }
    }

    auto meta_file = nh.param("metadata", std::string{});
    if (!meta_file.size()) {
        if (replay) {
            ROS_ERROR("Must specify metadata file in replay mode");
        } else {
            ROS_ERROR("Must specify path for metadata output");
        }
        return EXIT_FAILURE;
    }

    if (!replay && !hostname.size()) {
        ROS_ERROR("Must specify a sensor hostname");
        return EXIT_FAILURE;
    }

    ROS_INFO("Client version: %s", ouster::CLIENT_VERSION_FULL);

    if (replay) {
        ROS_INFO("Running in replay mode");

        // populate info for config service
        try {
            auto info = sensor::metadata_from_json(meta_file);
            published_metadata = to_string(info);

            ROS_INFO("Using lidar_mode: %s",
                     sensor::to_string(info.mode).c_str());
            ROS_INFO("%s sn: %s firmware rev: %s", info.prod_line.c_str(),
                     info.sn.c_str(), info.fw_rev.c_str());

            // just serve config service
            ros::spin();
            return EXIT_SUCCESS;
        } catch (const std::runtime_error& e) {
            ROS_ERROR("Error when running in replay mode: %s", e.what());
        }
    } else {
        if (udp_dest.size()) {
            ROS_INFO("Sending UDP data to %s", udp_dest.c_str());
        } else {
            ROS_INFO("Using automatic UDP destination");
        }
        ROS_INFO("Waiting for sensor %s to initialize ...", hostname.c_str());

        auto cli = sensor::init_client(hostname, udp_dest, lidar_mode,
                                       timestamp_mode, lidar_port, imu_port);

        if (!cli) {
            ROS_ERROR("Failed to initialize sensor at: %s", hostname.c_str());
            return EXIT_FAILURE;
        }
        ROS_INFO("Sensor initialized successfully");

        // write metadata file. If metadata_path is relative, will use cwd
        // (usually ~/.ros)
        auto metadata = sensor::get_metadata(*cli);
        if (!write_metadata(meta_file, metadata)) {
            ROS_ERROR("Exiting because of failure to write metadata path");
            return EXIT_FAILURE;
        }

        // populate sensor info
        auto info = sensor::parse_metadata(metadata);
        populate_metadata_defaults(info, sensor::MODE_UNSPEC);
        published_metadata = to_string(info);

        ROS_INFO("Using lidar_mode: %s", sensor::to_string(info.mode).c_str());
        ROS_INFO("%s sn: %s firmware rev: %s", info.prod_line.c_str(),
                 info.sn.c_str(), info.fw_rev.c_str());

        // publish packet messages from the sensor
        return connection_loop(nh, *cli, info);
    }
}
