/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * An example that demonstrates how to use mouse events with images.
 */

#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <random>

#include "ouster/impl/build.h"
#include "ouster/point_viz.h"

using namespace ouster;
using namespace ouster::viz;
using namespace std::placeholders;

constexpr int w = 16;
constexpr int h = 8;

float img_data[w * h];
auto img = std::make_shared<Image>();

bool set_pixel_from_window_coordinates(ouster::viz::PointViz& viz,
                                       const WindowCtx& ctx, double x,
                                       double y) {
    auto pixel = img->window_coordinates_to_image_pixel(ctx, x, y);
    if (pixel) {
        img_data[pixel->first + pixel->second * w] = 1.0;
        img->set_image(w, h, img_data);
        viz.update();
        return false;
    }
    return true;
}

bool mouse_button_handler(ouster::viz::PointViz& viz, const WindowCtx& ctx,
                          MouseButton button, MouseButtonEvent event,
                          EventModifierKeys /*mods*/) {
    if (event == MouseButtonEvent::MOUSE_BUTTON_PRESSED &&
        button == MouseButton::MOUSE_BUTTON_LEFT) {
        return set_pixel_from_window_coordinates(viz, ctx, ctx.mouse_x,
                                                 ctx.mouse_y);
    }
    return true;
}

bool mouse_pos_handler(ouster::viz::PointViz& viz, const WindowCtx& ctx,
                       double x, double y) {
    if (ctx.lbutton_down) {
        return set_pixel_from_window_coordinates(viz, ctx, x, y);
    }
    return true;
}

int main(int argc, char*[]) {
    if (argc != 1) {
        std::cerr << "Version: " << ouster::SDK_VERSION_FULL << " ("
                  << ouster::BUILD_SYSTEM << ")"
                  << "\n\nUsage: viz_events_example" << std::endl;

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
    std::uniform_real_distribution<float> dis3(0, 1.0f);
    for (int i = 0; i < w * h; i++) {
        img_data[i] = 0.3 * dis3(re);
    }
    img->set_image(w, h, img_data);
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
