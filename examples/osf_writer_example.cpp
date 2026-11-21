/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 *
 * This file contains example code for working with the osf::Writer class of
 * the C++ Ouster SDK. Please see the sdk docs at docs.ouster.com for clearer
 * explanations.
 */

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ouster/core/camera_frame.h"
#include "ouster/core/camera_info.h"
#include "ouster/core/class_map.h"
#include "ouster/core/field.h"
#include "ouster/core/impl/build.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/lidar_info.h"
#include "ouster/core/object.h"
#include "ouster/core/open_source.h"
#include "ouster/core/sensor_info.h"
#include "ouster/osf/osf_frame_set_source.h"
// NOLINTBEGIN(google-build-using-namespace)
//! [doc-stag-osf-create-imports]
#include "ouster/osf/writer.h"
using namespace ouster::sdk;
//! [doc-etag-osf-create-imports]

void add_example_objects(core::LidarFrame& frame) {
    // clang-format off
    //! [doc-stag-add-object]
    using namespace ouster::sdk;

    std::vector<core::Object> objects(2);
    objects[0].id = 1;
    objects[0].creation_ts = 99;
    objects[0].timestamp = 199;
    objects[0].class_id = 1;
    objects[0].class_confidence = 0.9;
    objects[0].object_to_body.set_position(Eigen::Vector3d{1, 2, 3});
    objects[0].object_to_body.set_rotation(Eigen::Vector3d{2, 2, 2});
    objects[0].body_to_world.set_position(Eigen::Vector3d{10, 20, 30});
    objects[0].body_to_world.set_rotation(Eigen::Vector3d{0.1, 0.2, 0.3});
    objects[0].velocity = core::UnalignedVector3f{2, 3, 4};
    objects[0].dimensions = core::UnalignedVector3f{1, 1, 1};
    objects[0].properties["num_points"] = "[100]";
    objects[0].properties["attributes"] = "\[\"eats_icecream\", \"carries_bag\"]";
    objects[1].id = 2;
    objects[1].creation_ts = 100;
    objects[1].timestamp = 200;
    objects[1].class_id = 2;
    objects[1].class_confidence = 0.8;
    objects[1].object_to_body.set_position(Eigen::Vector3d{3, 2, 1});
    objects[1].object_to_body.set_rotation(Eigen::Vector3d{1, 1, 1});
    objects[1].body_to_world.set_position(Eigen::Vector3d{4, 5, 6});
    objects[1].body_to_world.set_rotation(Eigen::Vector3d{0.4, 0.5, 0.6});
    objects[1].velocity = core::UnalignedVector3f{4, 3, 2};
    objects[1].dimensions = core::UnalignedVector3f{2, 2, 2};
    objects[1].properties["num_points"] = "\"[50]\"";
    objects[1].properties["attributes"] = "[\"parked_illegaly\"]";
    frame.objects()["test_objects"] = objects;
    //! [doc-etag-add-object]
    // clang-format on
}

namespace {
void write_classmaps_example(const std::string& output_osf_filename,
                             std::shared_ptr<core::SensorInfo> sensor_info) {
    // clang-format off
    //! [doc-stag-write-classmaps]
    osf::Writer writer(output_osf_filename, {sensor_info});

    core::ClassMap class_map1;
    class_map1.class_map.emplace(1,
                                 "dog");
    class_map1.class_map.emplace(2,
                                 "cat");
    core::ClassMap class_map2;
    class_map2.class_map.emplace(1,
                                 "tree");
    class_map2.class_map.emplace(2,
                                 "bush");
    core::ClassMapSet class_maps;
    class_maps.class_maps.emplace("four_legs",
                                  class_map1);
    class_maps.class_maps.emplace("zero_legs",
                                  class_map2);
    core::FrameSetSourceMetadataSet frame_set_source_metadata_set;
    frame_set_source_metadata_set.entries.emplace("class_maps", class_maps);
    std::string additional_info = "Test additional info";
    frame_set_source_metadata_set.entries.emplace("additional_info", additional_info);

    writer.save(frame_set_source_metadata_set);
    //! [doc-etag-write-classmaps]
    // clang-format on
    writer.close();
}
}  // namespace

std::string read_classmaps_example(const std::string& osf_path) {
    //! [doc-stag-read-classmaps]
    using namespace ouster::sdk;
    auto src = open_source(osf_path);
    // In C++, FrameSetSource metadata uses type erasure to allow storing
    // different types of metadata So the type must be checked and the value
    // casted to the appropriate type.
    if (!src.metadata("class_maps").is<core::ClassMapSet>()) {
        throw std::runtime_error("Unexpected metadata type!");
    }
    auto class_map_set = src.metadata("class_maps").as<core::ClassMapSet>();
    auto dog_class = class_map_set.class_maps["four_legs"].class_map[1];
    //! [doc-etag-read-classmaps]
    return dog_class;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Version: " << SDK_VERSION_FULL << " (" << BUILD_SYSTEM << ")"
                  << "\n\nUsage: osf_writer_example <osf_file>" << std::endl;

        return (argc == 1) ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    const std::string osf_file = argv[1];
    std::cerr << "Writing lidar and camera frames to OSF..." << '\n';
    //! [doc-stag-osf-create]
    // A single OSF file can hold multiple sensors of mixed kinds. Pass the
    // infos as a vector of SensorInfo base pointers; the Writer creates a lidar
    // or camera stream for each, in the order provided.
    auto lidar_info = core::LidarInfo::from_default(core::LidarMode::_512x10);
    auto camera_info = std::make_shared<core::CameraInfo>();
    camera_info->format.columns_per_frame = 32;
    camera_info->format.pixels_per_column = 16;
    std::vector<std::shared_ptr<core::SensorInfo>> infos{lidar_info, camera_info};
    osf::Writer writer(osf_file, infos);

    // Frames are addressed by a single, unified stream index: the sensor's
    // position in the registration order above (lidar at 0, camera at 1),
    // which is also the order sensor_info() reports on read. The frame's kind
    // must match the sensor registered at that index.

    // Instantiate a lidar frame that matches the LidarInfo dimensions and
    // attach some example objects. The lidar was registered first, so its
    // unified stream index is 0.
    auto lidar_frame = core::LidarFrame(lidar_info);
    add_example_objects(lidar_frame);
    writer.save(0, lidar_frame);

    // Instantiate a matching camera frame carrying a single RANGE image. The
    // camera was registered second, so its unified stream index is 1.
    auto camera_frame = core::CameraFrame(camera_info);
    camera_frame.w = camera_info->w();
    camera_frame.h = camera_info->h();
    camera_frame.add_field("RANGE", core::fd_array<uint32_t>(camera_info->h(), camera_info->w()));
    camera_frame.field<uint32_t>("RANGE").setConstant(1000u);
    camera_frame.timestamp()[0] = 1;
    writer.save(1, camera_frame);
    //! [doc-etag-osf-create]
    writer.close();

    std::cerr << "Writing ClassMap metadata example OSF..." << '\n';
    const std::string meta_osf = osf_file + ".meta.osf";
    write_classmaps_example(meta_osf, lidar_info);

    std::cerr << "Reading ClassMap metadata back from OSF..." << '\n';
    const auto dog_class = read_classmaps_example(meta_osf);
    std::cerr << "dog class: " << dog_class << '\n';

    return EXIT_SUCCESS;
}
// NOLINTEND(google-build-using-namespace)
