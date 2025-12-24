#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "ouster/mesh.h"
#include "ouster/point_viz.h"
#include "ouster/typedefs.h"

using namespace ouster::sdk;

int main(void) {
    // Initialize visualizer and add keyboard/mouse callbacks
    viz::PointViz viz("Viz example");
    viz::add_default_controls(viz);

    // Make the window visible
    viz.visible(true);
    viz.notifications_enabled = true;

    constexpr float M_2PI = 2.0f * M_PI;
    auto start = std::chrono::steady_clock::now();

    // Show quick welcome notification with default duration (2.0 seconds)
    viz.set_notification("Hello!");

    // For longer messages that need more reading time, use a custom duration
    double custom_notification_duration = 3.0;
    std::array<std::string, 3> messages = {
        "Press ESC to quit.", "Notifications display in the top-right corner.",
        "The cuboid rotates and changes color."};

    viz::vec4f rgba = {1.0f, 0.0f, 0.0f, 1.0f};
    core::mat4d transform = core::mat4d::Identity();
    auto cuboid_ptr =
        std::make_shared<viz::Cuboid>(core::mat4d_to_array(transform), rgba);
    viz.add(cuboid_ptr);

    while (viz.running()) {
        auto t = std::chrono::duration<float>(std::chrono::steady_clock::now() -
                                              start)
                     .count();
        auto seconds_since_start = static_cast<int>(t);
        if (!viz.notification_active()) {
            // Cycle through multiple instructional messages
            viz.set_notification(
                messages[static_cast<int>(seconds_since_start /
                                          custom_notification_duration) %
                         messages.size()],
                custom_notification_duration);
        }

        while (t > M_2PI) {
            t -= M_2PI;
        }

        // Rotate around Z axis
        double scale = 4.0 * cos(t) + 8.0;
        transform.block<3, 3>(0, 0) =
            scale *
            Eigen::AngleAxisd(static_cast<double>(t), Eigen::Vector3d::UnitZ())
                .toRotationMatrix();
        cuboid_ptr->set_transform(core::mat4d_to_array(transform));

        // Cycle through colors
        cuboid_ptr->set_rgba({(std::sin(t) + 1.0f) / 2.0f,
                              (std::sin(t / 2.0f) + 1.0f) / 2.0f,
                              (std::cos(t) + 1.0f) / 2.0f, 1.0f});

        // Update the viz and run one iteration of the event loop
        viz.update();
        viz.run_once();
    }
}
