/**
 * @file
 * @brief Example node to publish raw OS-1 output on ROS topics
 *
 * ROS Parameters
 * os1_hostname: hostname or IP in dotted decimal form of the sensor
 * os1_udp_dest: hostname or IP where the sensor will send data packets
 * os1_lidar_port: port to which the sensor should send lidar data
 * os1_imu_port: port to which the sensor should send imu data
 */

#include <ros/console.h>
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <string>

#include "ouster/os1_packet.h"
#include "ouster/os1_util.h"
#include "ouster_ros/OS1ConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/os1_ros.h"

using PacketMsg = ouster_ros::PacketMsg;
using OS1ConfigSrv = ouster_ros::OS1ConfigSrv;
namespace OS1 = ouster::OS1;

// fill in values that could not be parsed from metadata
void populate_metadata_defaults(OS1::sensor_info& info,
                                const std::string& specified_lidar_mode) {
    if (!info.hostname.size()) info.hostname = "UNKNOWN";

    if (!info.sn.size()) info.sn = "UNKNOWN";

    OS1::version v = OS1::version_of_string(info.fw_rev);
    if (v == OS1::invalid_version)
        ROS_WARN("Unknown sensor firmware version; output may not be reliable");
    else if (v < OS1::min_version)
        ROS_WARN("Firmware < %s not supported; output may not be reliable",
                 to_string(OS1::min_version).c_str());

    if (!info.mode) {
        ROS_WARN(
            "Lidar mode not found in metadata; output may not be reliable");
        info.mode = OS1::lidar_mode_of_string(specified_lidar_mode);
    }

    if (info.beam_azimuth_angles.empty() || info.beam_altitude_angles.empty()) {
        ROS_WARN("Beam angles not found in metadata; using design values");
        info.beam_azimuth_angles = OS1::beam_azimuth_angles;
        info.beam_altitude_angles = OS1::beam_altitude_angles;
    }

    if (info.imu_to_sensor_transform.empty() ||
        info.lidar_to_sensor_transform.empty()) {
        ROS_WARN("Frame transforms not found in metadata; using design values");
        info.imu_to_sensor_transform = OS1::imu_to_sensor_transform;
        info.lidar_to_sensor_transform = OS1::lidar_to_sensor_transform;
    }
}

// try to read metadata file
std::string read_metadata(const std::string& meta_file) {
    if (meta_file.size()) {
        ROS_INFO("Reading metadata from %s", meta_file.c_str());
    } else {
        ROS_WARN("No metadata file specified");
        return "";
    }

    std::stringstream buf{};
    std::ifstream ifs{};
    ifs.open(meta_file);
    buf << ifs.rdbuf();
    ifs.close();

    if (!ifs)
        ROS_WARN("Failed to read %s; check that the path is valid",
                 meta_file.c_str());

    return buf.str();
}

// try to write metadata file
void write_metadata(const std::string& meta_file, const std::string& metadata) {
    std::ofstream ofs;
    ofs.open(meta_file);
    ofs << metadata << std::endl;
    ofs.close();
    if (ofs) {
        ROS_INFO("Wrote metadata to %s", meta_file.c_str());
    } else {
        ROS_WARN("Failed to write metadata to %s; check that the path is valid",
                 meta_file.c_str());
    }
}

int connection_loop(ros::NodeHandle& nh, OS1::client& cli) {
    auto lidar_packet_pub = nh.advertise<PacketMsg>("lidar_packets", 1280);
    auto imu_packet_pub = nh.advertise<PacketMsg>("imu_packets", 100);

    PacketMsg lidar_packet, imu_packet;
    lidar_packet.buf.resize(OS1::lidar_packet_bytes + 1);
    imu_packet.buf.resize(OS1::imu_packet_bytes + 1);

    while (ros::ok()) {
        auto state = OS1::poll_client(cli);
        if (state == OS1::EXIT) {
            ROS_INFO("poll_client: caught signal, exiting");
            return EXIT_SUCCESS;
        }
        if (state & OS1::ERROR) {
            ROS_ERROR("poll_client: returned error");
            return EXIT_FAILURE;
        }
        if (state & OS1::LIDAR_DATA) {
            if (OS1::read_lidar_packet(cli, lidar_packet.buf.data()))
                lidar_packet_pub.publish(lidar_packet);
        }
        if (state & OS1::IMU_DATA) {
            if (OS1::read_imu_packet(cli, imu_packet.buf.data()))
                imu_packet_pub.publish(imu_packet);
        }
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "os1_node");
    ros::NodeHandle nh("~");

    OS1::sensor_info info{};
    auto srv =
        nh.advertiseService<OS1ConfigSrv::Request, OS1ConfigSrv::Response>(
            "os1_config",
            [&](OS1ConfigSrv::Request&, OS1ConfigSrv::Response& res) {
                res.hostname = info.hostname;
                res.lidar_mode = to_string(info.mode);
                res.beam_azimuth_angles = info.beam_azimuth_angles;
                res.beam_altitude_angles = info.beam_altitude_angles;
                res.imu_to_sensor_transform = info.imu_to_sensor_transform;
                res.lidar_to_sensor_transform = info.lidar_to_sensor_transform;
                return true;
            });

    // empty indicates "not set" since roslaunch xml can't optionally set params
    auto hostname = nh.param("os1_hostname", std::string{});
    auto udp_dest = nh.param("os1_udp_dest", std::string{});
    auto lidar_port = nh.param("os1_lidar_port", 0);
    auto imu_port = nh.param("os1_imu_port", 0);
    auto replay = nh.param("replay", false);
    auto lidar_mode = nh.param("lidar_mode", std::string{});
    auto timestamp_mode = nh.param("timestamp_mode", std::string{});

    // fall back to metadata file name based on hostname, if available
    auto meta_file = nh.param("metadata", std::string{});
    if (!meta_file.size() && hostname.size()) meta_file = hostname + ".json";

    if (lidar_mode.size()) {
        if (replay) ROS_WARN("Lidar mode set in replay mode. May be ignored");
    } else {
        lidar_mode = OS1::to_string(OS1::MODE_1024x10);
    }

    if (!OS1::lidar_mode_of_string(lidar_mode)) {
        ROS_ERROR("Invalid lidar mode %s", lidar_mode.c_str());
        return EXIT_FAILURE;
    }

    if (not timestamp_mode.size()) {
        timestamp_mode = OS1::to_string(OS1::TIME_FROM_INTERNAL_OSC);
    }

    if (!OS1::timestamp_mode_of_string(timestamp_mode)) {
        ROS_ERROR("Invalid timestamp mode %s", timestamp_mode.c_str());
        return EXIT_FAILURE;
    }

    if (!replay && (!hostname.size() || !udp_dest.size())) {
        ROS_ERROR("Must specify both hostname and udp destination");
        return EXIT_FAILURE;
    }

    if (replay) {
        ROS_INFO("Running in replay mode");

        // populate info for config service
        std::string metadata = read_metadata(meta_file);
        info = OS1::parse_metadata(metadata);
        populate_metadata_defaults(info, lidar_mode);

        ROS_INFO("Using lidar_mode: %s", OS1::to_string(info.mode).c_str());
        ROS_INFO("Sensor sn: %s firmware rev: %s", info.sn.c_str(),
                 info.fw_rev.c_str());

        // just serve config service
        ros::spin();
        return EXIT_SUCCESS;
    } else {
        ROS_INFO("Connecting to sensor at %s...", hostname.c_str());

        ROS_INFO("Sending data to %s using lidar_mode: %s", udp_dest.c_str(),
                 lidar_mode.c_str());

        auto cli = OS1::init_client(hostname, udp_dest,
                                    OS1::lidar_mode_of_string(lidar_mode),
                                    OS1::timestamp_mode_of_string(timestamp_mode),
                                    lidar_port, imu_port);

        if (!cli) {
            ROS_ERROR("Failed to initialize sensor at: %s", hostname.c_str());
            return EXIT_FAILURE;
        }
        ROS_INFO("Sensor reconfigured successfully, waiting for data...");

        // write metadata file to cwd (usually ~/.ros)
        auto metadata = OS1::get_metadata(*cli);
        write_metadata(meta_file, metadata);

        // populate sensor info
        info = OS1::parse_metadata(metadata);
        populate_metadata_defaults(info, "");

        ROS_INFO("Sensor sn: %s firmware rev: %s", info.sn.c_str(),
                 info.fw_rev.c_str());

        // publish packet messages from the sensor
        return connection_loop(nh, *cli);
    }
}
