/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * Minimal static point viz library example.
 */
#include <algorithm>
#include <iostream>
#include <memory>
#include <random>

#include "ouster/impl/build.h"
#include "ouster/point_viz.h"

using namespace ouster;

int main(int argc, char*[]) {
    if (argc != 1) {
        std::cerr << "Version: " << ouster::SDK_VERSION_FULL << " ("
                  << ouster::BUILD_SYSTEM << ")"
                  << "\n\nUsage: viz_example" << std::endl;

        return EXIT_FAILURE;
    }

    // std::random boilerplate
    std::random_device rd;
    std::default_random_engine re(rd());
    std::uniform_real_distribution<float> dis(-20.0, 20.0);
    std::uniform_real_distribution<float> dis2(0.0, 1.0);

    // number of points to display
    const size_t cloud_size = 1024;

    // populate random coordinates and color indices
    std::vector<float> points(3 * cloud_size);
    std::generate(points.begin(), points.end(), [&]() { return dis(re); });

    std::vector<float> colors(cloud_size);
    std::generate(colors.begin(), colors.end(), [&]() { return dis2(re); });

    // initialize visualizer and add keyboard/mouse callbacks
    ouster::viz::PointViz viz("Viz example");
    ouster::viz::add_default_controls(viz);

    // create a point cloud and register it with the visualizer
    auto cloud = std::make_shared<ouster::viz::Cloud>(cloud_size);
    viz.add(cloud);

    // update visualizer cloud object
    cloud->set_xyz(points.data());
    cloud->set_key(colors.data());

    // send updates to be rendered. This method is thread-safe
    viz.update();

    // run rendering loop. Will return when the window is closed
    std::cout << "Running rendering loop: press ESC to exit" << std::endl;
    viz.run();
    std::cout << "Window closed, exiting" << std::endl;

    return EXIT_SUCCESS;
}
