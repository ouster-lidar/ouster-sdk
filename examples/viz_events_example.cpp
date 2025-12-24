/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * An example that demonstrates how to use mouse events with images.
 */

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include "ouster/impl/build.h"
#include "ouster/point_viz.h"

using namespace ouster::sdk;
using namespace std::placeholders;

constexpr int IMAGE_WIDTH = 16;
constexpr int IMAGE_HEIGHT = 8;

float img_data[IMAGE_WIDTH * IMAGE_HEIGHT];
auto img = std::make_shared<viz::Image>();

// returns true if the provided window coordinates were within the image
bool set_pixel_from_viewport_coordinates(viz::PointViz& viz,
                                         const viz::WindowCtx& ctx, double x,
                                         double y) {
    auto pixel = img->viewport_coordinates_to_image_pixel(ctx, x, y);
    if (pixel.first >= 0 && pixel.first < IMAGE_WIDTH && pixel.second >= 0 &&
        pixel.second < IMAGE_HEIGHT) {
        img_data[pixel.first + (pixel.second * IMAGE_WIDTH)] = 1.0;
        img->set_image(IMAGE_WIDTH, IMAGE_HEIGHT, img_data);
        viz.update();
        return false;
    }
    return true;
}

bool mouse_button_handler(viz::PointViz& viz, const viz::WindowCtx& ctx,
                          viz::MouseButton button, viz::MouseButtonEvent event,
                          viz::EventModifierKeys /*mods*/) {
    if (event == viz::MouseButtonEvent::MOUSE_BUTTON_PRESSED &&
        button == viz::MouseButton::MOUSE_BUTTON_LEFT) {
        return set_pixel_from_viewport_coordinates(viz, ctx, ctx.mouse_x,
                                                   ctx.mouse_y);
    }
    return true;
}

bool mouse_pos_handler(viz::PointViz& viz, const viz::WindowCtx& ctx, double x,
                       double y) {
    if (ctx.lbutton_down) {
        return set_pixel_from_viewport_coordinates(viz, ctx, x, y);
    }
    return true;
}

int main(int argc, char* /*unused*/[]) {
    if (argc != 1) {
        std::cerr << "Version: " << SDK_VERSION_FULL << " (" << BUILD_SYSTEM
                  << ")"
                  << "\n\nUsage: viz_events_example" << std::endl;

        return EXIT_FAILURE;
    }

    // std::random boilerplate
    std::random_device random_device;
    std::default_random_engine random_engine(random_device());
    std::uniform_real_distribution<float> dis(-20.0, 20.0);
    std::uniform_real_distribution<float> dis2(0.0, 1.0);

    // number of points to display
    const size_t cloud_size = 1024;

    // populate random coordinates and color indices
    std::vector<float> points(3 * cloud_size);
    std::generate(points.begin(), points.end(),
                  [&]() { return dis(random_engine); });

    std::vector<float> colors(cloud_size);
    std::generate(colors.begin(), colors.end(),
                  [&]() { return dis2(random_engine); });

    // initialize visualizer and add keyboard/mouse callbacks
    viz::PointViz viz("Viz example");
    viz::add_default_controls(viz);

    // create a point cloud and register it with the visualizer
    auto cloud = std::make_shared<viz::Cloud>(cloud_size);
    viz.add(cloud);

    // update visualizer cloud object
    cloud->set_xyz(points.data());
    cloud->set_key(colors.data());

    // send updates to be rendered. This method is thread-safe
    std::uniform_real_distribution<float> dis3(0, 1.0f);
    for (int i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; i++) {
        img_data[i] = 0.3 * dis3(random_engine);
    }
    img->set_image(IMAGE_WIDTH, IMAGE_HEIGHT, img_data);
    img->set_position(-0.75, 0.5, -0.25, 0.5);
    img->set_hshift(0.6);

    auto mouse_pos_handler_fn =
        std::bind(mouse_pos_handler, std::ref(viz), _1, _2, _3);
    viz.push_mouse_pos_handler(mouse_pos_handler_fn);
    auto mouse_button_handler_fn =
        std::bind(mouse_button_handler, std::ref(viz), _1, _2, _3, _4);
    viz.push_mouse_button_handler(mouse_button_handler_fn);

    viz.add(img);
    viz.update();

    // run rendering loop. Will return when the window is closed
    std::cout << "Running rendering loop: press ESC to exit" << std::endl;
    viz.run();
    std::cout << "Window closed, exiting" << std::endl;

    return EXIT_SUCCESS;
}
