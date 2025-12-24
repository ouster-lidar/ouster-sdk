#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <string>
#include <utility>
#include <vector>

#include "ouster/beam_config.h"
#include "ouster/mesh.h"
#include "ouster/typedefs.h"
#include "ouster/types.h"
#include "ouster/zone.h"

using ouster::sdk::core::BeamConfig;
using ouster::sdk::core::mat4d;
using ouster::sdk::core::mat4d_from_array;
using ouster::sdk::core::Stl;
using ouster::sdk::core::Zone;

namespace {
void error(const std::string& err) {
    std::cerr << err << std::endl;
    exit(-1);  // NOLINT(concurrency-mt-unsafe)
}
}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 5) {
        error(
            "Usage: zone_render_mini [sensor info json file] [zone monitor "
            "json file] [stl file] [output zrb file]");
    }
    std::string sensor_info_path(argv[1]);
    std::string zone_monitor_json_path(argv[2]);
    std::string stl_path(argv[3]);
    std::string zrb_path(argv[4]);

    ouster::sdk::core::SensorInfo sensor_info =
        ouster::sdk::core::metadata_from_json(sensor_info_path);

    std::ifstream zone_monitor_json_file(zone_monitor_json_path);
    auto zone_monitor_json = jsoncons::json::parse(zone_monitor_json_file);

    mat4d sensor_to_body_transform = mat4d_from_array(
        zone_monitor_json["sensor_to_body_transform"]
            .as<std::array<double, mat4d::SizeAtCompileTime>>());

    float m_per_zmbin = ouster::sdk::core::DEFAULT_M_PER_ZMBIN;
    BeamConfig beam_config(
        sensor_info.w(), sensor_info.beam_altitude_angles,
        sensor_info.beam_azimuth_angles, sensor_info.beam_to_lidar_transform,
        sensor_info.lidar_to_sensor_transform, sensor_to_body_transform,
        m_per_zmbin, sensor_info.sn);

    Zone zone;
    zone.point_count =
        zone_monitor_json["zones"]["0"]["point_count"].as<uint16_t>();
    zone.frame_count =
        zone_monitor_json["zones"]["0"]["frame_count"].as<uint16_t>();
    Zone::string_to_zonemode(
        zone_monitor_json["zones"]["0"]["mode"].as<std::string>(), zone.mode);
    auto stl = Stl(stl_path);

    Stl::string_to_coordinate_frame(
        zone_monitor_json["zones"]["0"]["stl"]["coordinate_frame"]
            .as<std::string>(),
        stl.coordinate_frame);
    zone.stl = stl;
    if (zone.render(beam_config)) {
        zone.zrb->save(zrb_path);
    } else {
        std::cerr
            << "ZoneMon: render failed or mesh was entirely out of view.\n";
    }
}
