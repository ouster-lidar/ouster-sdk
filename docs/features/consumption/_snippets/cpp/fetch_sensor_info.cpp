#include <fstream>
#include <iostream>
#include <string>

#include "ouster/core/types.h"
#include "ouster/sensor/sensor_frame_set_source.h"

using namespace ouster::sdk;

void fetch_sensor_info(const std::string& sensor_hostname) {
    // clang-format off
    //! [doc-stag-fetch-sensor-info]
    sensor::SensorFrameSetSource source(sensor_hostname);
    auto sensor_info = source.sensor_info()[0];
    std::cout << "Retrieved sensor info:\n";
    std::cout << "  serial no:        " << sensor_info->sn << "\n";
    std::cout << "  firmware version: " << sensor_info->fw_rev << "\n";
    std::cout << "  product line:     " << sensor_info->prod_line << "\n";
    std::cout << "  lidar mode:       " << core::to_string(sensor_info->config.lidar_mode.value()) << "\n";
    std::cout << "  columns/frame:    " << sensor_info->format.columns_per_frame << "\n";
    std::cout << "  beam angles:      " << sensor_info->beam_altitude_angles.size() << "\n";
    std::cout << "  altitude:         " << sensor_info->beam_azimuth_angles.size() << "\n";
    std::cout << "Writing to " << sensor_hostname << ".json\n";
    std::ofstream out(sensor_hostname + ".json");
    out << sensor_info->to_json_string();
    //! [doc-etag-fetch-sensor-info]
    // clang-format on
}
