/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_sensor_nodelet.cpp
 * @brief A nodelet that connects to a live ouster sensor
 */

#include <pluginlib/class_list_macros.h>

#include <fstream>
#include <string>

#include "ouster_ros/GetConfig.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/SetConfig.h"
#include "ouster_ros/os_client_base_nodelet.h"

namespace sensor = ouster::sensor;
using nonstd::optional;
using ouster_ros::GetConfig;
using ouster_ros::PacketMsg;
using ouster_ros::SetConfig;

namespace nodelets_os {

class OusterSensor : public OusterClientBase {
   private:
    virtual void onInit() override {
        auto& pnh = getPrivateNodeHandle();
        sensor_client = create_client(pnh);
        auto sensor_conf = create_sensor_config_rosparams(pnh, *sensor_client);
        configure_sensor(hostname, sensor_conf.first, sensor_conf.second);
        update_config_and_metadata(*sensor_client);
        save_metadata(pnh);
        OusterClientBase::onInit();
        create_get_config_service();
        create_set_config_service();
        start_connection_loop();
    }

    bool update_config_and_metadata(sensor::client& cli) {
        sensor::sensor_config config;
        auto success = get_config(hostname, config);
        if (!success) {
            NODELET_ERROR("Failed to collect sensor config");
            cached_config.clear();
            cached_metadata.clear();
            return false;
        }

        cached_config = to_string(config);

        cached_metadata = sensor::get_metadata(cli);
        if (cached_metadata.empty()) {
            NODELET_ERROR("Failed to collect sensor metadata");
            return false;
        }

        info = sensor::parse_metadata(cached_metadata);
        // TODO: revist when *min_version* is changed
        populate_metadata_defaults(info, sensor::MODE_UNSPEC);
        display_lidar_info(info);
        
        return cached_config.size() > 0 && cached_metadata.size() > 0;
    }

    void save_metadata(ros::NodeHandle& nh) {
        auto meta_file = nh.param("metadata", std::string{});
        if (!meta_file.size()) {
            meta_file =
                hostname.substr(0, hostname.rfind('.')) + "-metadata.json";
            NODELET_WARN_STREAM(
                "No metadata file was specified, using: " << meta_file);
        }

        // write metadata file. If metadata_path is relative, will use cwd
        // (usually ~/.ros)
        if (!write_metadata(meta_file, cached_metadata)) {
            NODELET_ERROR("Exiting because of failure to write metadata path");
            throw std::runtime_error("Failure to write metadata path");
        }
    }

    void create_get_config_service() {
        auto& nh = getNodeHandle();
        get_config_srv =
            nh.advertiseService<GetConfig::Request, GetConfig::Response>(
                "get_config",
                [&](GetConfig::Request&, GetConfig::Response& response) {
                    response.config = cached_config;
                    return cached_config.size() > 0;
                });

        NODELET_INFO("get_config service created");
    }

    void create_set_config_service() {
        auto& nh = getNodeHandle();
        set_config_srv =
            nh.advertiseService<SetConfig::Request, SetConfig::Response>(
                "set_config", [&](SetConfig::Request& request,
                                  SetConfig::Response& response) {
                    sensor::sensor_config config;
                    response.config = "";
                    auto success =
                        load_config_file(request.config_file, config);
                    if (!success) {
                        NODELET_ERROR_STREAM("Failed to load and parse file: "
                                             << request.config_file);
                        return false;
                    }

                    try {
                        configure_sensor(hostname, config, 0);
                    } catch (const std::runtime_error& e) {
                        return false;
                    } catch (const std::invalid_argument& ia) {
                        return false;
                    }
                    success = update_config_and_metadata(*sensor_client);
                    response.config = cached_config;
                    return success;
                });

        NODELET_INFO("set_config service created");
    }

    std::shared_ptr<sensor::client> create_client(ros::NodeHandle& nh) {
        hostname = nh.param("sensor_hostname", std::string{});
        auto lidar_port = nh.param("lidar_port", 0);
        auto imu_port = nh.param("imu_port", 0);

        if (!hostname.size()) {
            NODELET_ERROR("Must specify a sensor hostname");
            throw std::runtime_error("sensor hostname not specified!");
        }

        NODELET_INFO("Starting sensor %s initialization...", hostname.c_str());

        // use no-config version of init_client to allow for random ports
        auto cli = sensor::init_client(hostname, lidar_port, imu_port);
        if (!cli) {
            NODELET_ERROR("Failed to initialize client");
            throw std::runtime_error("Failed to initialize client");
        }

        return cli;
    }

    std::pair<sensor::sensor_config, int> create_sensor_config_rosparams(
        ros::NodeHandle& nh, sensor::client& cli) {
        auto udp_dest = nh.param("udp_dest", std::string{});
        auto lidar_mode_arg = nh.param("lidar_mode", std::string{});
        auto timestamp_mode_arg = nh.param("timestamp_mode", std::string{});

        std::string udp_profile_lidar_arg;
        nh.param<std::string>("udp_profile_lidar", udp_profile_lidar_arg, "");

        optional<sensor::UDPProfileLidar> udp_profile_lidar;
        if (udp_profile_lidar_arg.size()) {
            // set lidar profile from param
            udp_profile_lidar =
                sensor::udp_profile_lidar_of_string(udp_profile_lidar_arg);
            if (!udp_profile_lidar) {
                NODELET_ERROR("Invalid udp profile lidar: %s",
                              udp_profile_lidar_arg.c_str());
                throw std::runtime_error("Invalid udp profile");
            }
        }

        // set lidar mode from param
        sensor::lidar_mode lidar_mode = sensor::MODE_UNSPEC;
        if (lidar_mode_arg.size()) {
            lidar_mode = sensor::lidar_mode_of_string(lidar_mode_arg);
            if (!lidar_mode) {
                NODELET_ERROR("Invalid lidar mode %s", lidar_mode_arg.c_str());
                throw std::runtime_error("Invalid lidar mode");
            }
        }

        // set timestamp mode from param
        sensor::timestamp_mode timestamp_mode = sensor::TIME_FROM_UNSPEC;
        if (timestamp_mode_arg.size()) {
            timestamp_mode =
                sensor::timestamp_mode_of_string(timestamp_mode_arg);
            if (!timestamp_mode) {
                NODELET_ERROR("Invalid timestamp mode %s",
                              timestamp_mode_arg.c_str());
                throw std::runtime_error("Invalid timestamp mode");
            }
        }

        sensor::sensor_config config;
        config.udp_port_imu = sensor::get_imu_port(cli);
        config.udp_port_lidar = sensor::get_lidar_port(cli);
        config.udp_profile_lidar = udp_profile_lidar;
        config.operating_mode = sensor::OPERATING_NORMAL;
        if (lidar_mode) config.ld_mode = lidar_mode;
        if (timestamp_mode) config.ts_mode = timestamp_mode;

        uint8_t config_flags = 0;

        if (udp_dest.size()) {
            NODELET_INFO("Will send UDP data to %s", udp_dest.c_str());
            config.udp_dest = udp_dest;
        } else {
            NODELET_INFO("Will use automatic UDP destination");
            config_flags |= ouster::sensor::CONFIG_UDP_DEST_AUTO;
        }

        return std::make_pair(config, config_flags);
    }

    void configure_sensor(const std::string& hostname,
                          const sensor::sensor_config& config,
                          int config_flags) {
        try {
            if (!set_config(hostname, config, config_flags)) {
                auto err_msg = "Error connecting to sensor " + hostname;
                NODELET_ERROR_STREAM(err_msg);
                throw std::runtime_error(err_msg);
            }
        } catch (const std::runtime_error& e) {
            NODELET_ERROR("Error setting config:  %s", e.what());
            throw;
        } catch (const std::invalid_argument& ia) {
            NODELET_ERROR("Error setting config: %s", ia.what());
            throw;
        }

        NODELET_WARN_STREAM("Sensor " << hostname
                                      << " configured successfully");
    }

    bool load_config_file(const std::string& config_file,
                          sensor::sensor_config& out_config) {
        std::ifstream ifs{};
        ifs.open(config_file);
        if (ifs.fail()) return false;
        std::stringstream buf;
        buf << ifs.rdbuf();
        out_config = sensor::parse_config(buf.str());
        return true;
    }

   private:
    // fill in values that could not be parsed from metadata
    void populate_metadata_defaults(sensor::sensor_info& info,
                                    sensor::lidar_mode specified_lidar_mode) {
        if (!info.name.size()) info.name = "UNKNOWN";

        if (!info.sn.size()) info.sn = "UNKNOWN";

        ouster::util::version v = ouster::util::version_of_string(info.fw_rev);
        if (v == ouster::util::invalid_version)
            NODELET_WARN(
                "Unknown sensor firmware version; output may not be reliable");
        else if (v < sensor::min_version)
            NODELET_WARN(
                "Firmware < %s not supported; output may not be reliable",
                to_string(sensor::min_version).c_str());

        if (!info.mode) {
            NODELET_WARN(
                "Lidar mode not found in metadata; output may not be reliable");
            info.mode = specified_lidar_mode;
        }

        if (!info.prod_line.size()) info.prod_line = "UNKNOWN";

        if (info.beam_azimuth_angles.empty() ||
            info.beam_altitude_angles.empty()) {
            NODELET_ERROR(
                "Beam angles not found in metadata; using design values");
            info.beam_azimuth_angles = sensor::gen1_azimuth_angles;
            info.beam_altitude_angles = sensor::gen1_altitude_angles;
        }
    }

    // try to write metadata file
    bool write_metadata(const std::string& meta_file,
                        const std::string& metadata) {
        std::ofstream ofs;
        ofs.open(meta_file);
        ofs << metadata << std::endl;
        ofs.close();
        if (ofs) {
            NODELET_INFO("Wrote metadata to %s", meta_file.c_str());
        } else {
            NODELET_WARN(
                "Failed to write metadata to %s; check that the path is valid. "
                "If "
                "you provided a relative path, please note that the working "
                "directory of all ROS nodes is set by default to $ROS_HOME",
                meta_file.c_str());
            return false;
        }
        return true;
    }

    void start_connection_loop() {
        auto& nh = getNodeHandle();

        auto pf = sensor::get_format(info);
        lidar_packet.buf.resize(pf.lidar_packet_size + 1);
        imu_packet.buf.resize(pf.imu_packet_size + 1);

        lidar_packet_pub = nh.advertise<PacketMsg>("lidar_packets", 1280);
        imu_packet_pub = nh.advertise<PacketMsg>("imu_packets", 100);

        timer_ = nh.createTimer(
            ros::Duration(0),
            boost::bind(&OusterSensor::timer_callback, this, _1), true);
    }

    void connection_loop(sensor::client& cli, const sensor::sensor_info& info) {
        auto pf = sensor::get_format(info);

        auto state = sensor::poll_client(cli);
        if (state == sensor::EXIT) {
            NODELET_INFO("poll_client: caught signal, exiting");
            return;
        }
        if (state & sensor::CLIENT_ERROR) {
            NODELET_ERROR("poll_client: returned error");
            return;
        }
        if (state & sensor::LIDAR_DATA) {
            if (sensor::read_lidar_packet(cli, lidar_packet.buf.data(), pf))
                lidar_packet_pub.publish(lidar_packet);
        }
        if (state & sensor::IMU_DATA) {
            if (sensor::read_imu_packet(cli, imu_packet.buf.data(), pf))
                imu_packet_pub.publish(imu_packet);
        }
    }

    void timer_callback(const ros::TimerEvent&) {
        connection_loop(*sensor_client, info);
        timer_.stop();
        timer_.start();
    }

   private:
    PacketMsg lidar_packet;
    PacketMsg imu_packet;
    ros::Publisher lidar_packet_pub;
    ros::Publisher imu_packet_pub;
    std::shared_ptr<sensor::client> sensor_client;
    ros::Timer timer_;
    std::string hostname;
    ros::ServiceServer get_config_srv;
    ros::ServiceServer set_config_srv;
    std::string cached_config;
};

}  // namespace nodelets_os

PLUGINLIB_EXPORT_CLASS(nodelets_os::OusterSensor, nodelet::Nodelet)
